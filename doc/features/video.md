# Video Streaming

H.264 RTP video from a MIPI camera on the Pi to QGroundControl. Each camera appears as a separate MAVLink camera component; each sensor mode is advertised as a stream. QGC starts / stops streams via `MAV_CMD_VIDEO_START_STREAMING`.

Implementation: [`src/camera/`](../../src/camera/) + [`src/mavlink/camera_handlers.cpp`](../../src/mavlink/camera_handlers.cpp).

---

## Architecture

```
QGC
  ↓ HEARTBEAT, MAV_CMD_REQUEST_MESSAGE(CAMERA_INFORMATION), …
Pi daemon
  ├── heartbeat per camera component (MAV_COMP_ID_CAMERA..+i)
  ├── CameraDiscovery — parses `libcamera-hello --list-cameras`
  ├── GstPipeline — fork/exec gst-launch-1.0, killpg on stop
  └── stream watchdog — restart pipeline if it exits unexpectedly
      ↓
   gst-launch-1.0 libcamerasrc ! … ! v4l2h264enc ! rtph264pay ! udpsink
      ↓ H.264 RTP @ UDP 5600
   QGC video widget
```

- Each camera is advertised as its own MAVLink component (`MAV_COMP_ID_CAMERA + i`).
- Each sensor mode from the camera is its own `VIDEO_STREAM_INFORMATION` (stream IDs start at 1).
- `COMMAND_LONG` is routed by `target_component`: components ≥ 100 go to camera handlers, component 1 is the autopilot.
- Start / stop / watchdog-restart events are reported to QGC as `STATUSTEXT`.

See [../reference/mavlink-camera-protocol.md](../reference/mavlink-camera-protocol.md) for the MAVLink protocol details.

---

## Sensor modes (OV5647 / Pi Camera v1)

| Resolution | Max FPS | Sensor crop | FOV | Notes |
|---|---|---|---|---|
| 640 × 480 | 58.92 | (16,0) / 2560 × 1920 | Full 4:3 | Binned 4×, low CPU |
| **1296 × 972** | **46.34** | **(0,0) / 2592 × 1944** | **Full 4:3** | **Binned 2×, best for rover** |
| 1920 × 1080 | 32.81 | (348,434) / 1928 × 1080 | Center-crop 16:9 | Loses ~44 % of sensor area |
| 2592 × 1944 | 15.63 | (0,0) / 2592 × 1944 | Full 4:3 | Full res, too slow |

**Recommended:** 1296 × 972. It uses the full sensor with 2× binning, giving the widest FOV at a usable frame rate.

Stream names sent to QGC are `"<W>x<H> <fps>fps <crop>"`, e.g. `"1296x972 46.34fps full"`. `full` = full sensor, `crop` = centre crop.

---

## Runtime parameters

| Param | Default | Range | Effect |
|---|---|---|---|
| `VIDEO_BITRATE` | 5 000 000 | 25 000 – 25 000 000 | H.264 encoder target bitrate (bps) |
| `VIDEO_FPS` | 30 | 1 – 60 | Frame rate cap; lowered when the sensor mode's max fps is lower |

Change live from QGC; the running pipeline is restarted with the new setting.

---

## QGC IP detection and the `qgc_ip` file

The GStreamer pipeline sends H.264 RTP to `udp://<QGC-IP>:5600`. The daemon learns the QGC IP from the source address of the first MAVLink packet it receives. Through WireGuard this is normally QGC's LAN IP, which is correct.

If video never starts but MAVLink is connected, the IP detection may have misfired (e.g., the first packet came from a different source). Override with the `qgc_ip` file:

```bash
echo "192.168.50.100" > /home/pi/ground-rover-daemon/qgc_ip
sudo systemctl restart ground-rover-daemon
```

The file is read once at daemon startup. Remove it to revert to auto-detection.

---

## The GStreamer pipeline

For 1296 × 972 the daemon spawns:

```bash
gst-launch-1.0 libcamerasrc ! \
  video/x-raw,width=1296,height=972,framerate=30/1 ! \
  v4l2h264enc extra-controls="controls,repeat_sequence_header=1,video_bitrate=5000000" ! \
  video/x-h264,level=(string)4 ! \
  h264parse config-interval=-1 ! \
  rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=<QGC-IP> port=5600 sync=false
```

| Parameter | Purpose |
|---|---|
| `repeat_sequence_header=1` | SPS/PPS with every I-frame — lets the receiver join mid-stream |
| `config-interval=-1` (h264parse) | Forward SPS/PPS with every I-frame |
| `config-interval=1` (rtph264pay) | Include SPS/PPS in RTP for recovery |
| `mtu=1400` | Fits standard Ethernet MTU, avoids IP fragmentation |
| `sync=false` | No clock sync on udpsink — lower latency |
| `pt=96` | RTP payload type for H.264 |

---

## CPU cost

The Pi Zero 2W has 4 cores; the encoder is hardware-accelerated (`bcm2835-codec` on `/dev/video11`) but `libcamerasrc` does not export DMABuf, so every frame is CPU-copied from camera buffers to the encoder input.

| Resolution | CPU | Verdict |
|---|---|---|
| 640 × 480 | ~25 % | Plenty of headroom |
| **1296 × 972** | **~50 %** | **Good margin for the rover's workload** |
| 1920 × 1080 | ~100 % (1 core saturated) | Tight — may drop frames under load |

`rpicam-vid` (a separate binary) does use DMABuf internally and hits ~47 % at 1080p — an option if CPU becomes a problem.

---

## GPU memory

**`gpu_mem=128` is required.** Default 64 MB causes the encoder to fail at 1296 × 972 and above with:

```
Failed to process frame. Maybe due to not enough memory or failing driver
bcm2835_codec_start_streaming: Failed enabling i/p port, ret -3
```

This is configured in [../setup/01-raspberry-pi.md](../setup/01-raspberry-pi.md).

**Warning:** failed encode attempts can leave the codec driver in a broken state where even 640 × 480 fails. Recovery:

```bash
sudo modprobe -r bcm2835_codec && sudo modprobe bcm2835_codec
# or just reboot
```

---

## Latency

Measured end-to-end (Pi → QGC on the same LAN):

| Setup | Glass-to-glass |
|---|---|
| Direct GStreamer receive, `rtpjitterbuffer latency=0` | ~120–140 ms |
| QGC (default jitter buffer 200 ms + decode) | ~450 ms |

QGC adds ~300 ms on top of the network + codec baseline. For latency-critical bench testing, bypass QGC with a GStreamer receiver:

```bash
gst-launch-1.0 udpsrc port=5600 \
  caps="application/x-rtp,encoding-name=H264,payload=96" ! \
  rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! autovideosink sync=false
```

---

## Encoder controls

`v4l2-ctl -d /dev/video11 --list-ctrls` (on the Pi) lists the available encoder knobs. The daemon uses:

| Control | Value | Purpose |
|---|---|---|
| `video_bitrate` | from `VIDEO_BITRATE` | Target bitrate |
| `repeat_sequence_header` | 1 | SPS/PPS every I-frame |
| Profile | High (default) | H.264 profile — fine for QGC |

B-frames are not supported on this encoder (`video_b_frames=0` only).

---

## Discovery behavior

On startup, the daemon runs `libcamera-hello --list-cameras` once and caches the result. Each detected camera is registered as a MAVLink component and starts heartbeating. QGC's camera-discovery retries start immediately — the daemon answers with `CAMERA_INFORMATION` (capability flag `HAS_VIDEO_STREAM`) and then with one `VIDEO_STREAM_INFORMATION` per sensor mode.

If no camera is detected, the component is not advertised — QGC simply shows no camera icon.

---

## Wiring

See [../setup/03-wiring.md §3.4](../setup/03-wiring.md#34-mipi-camera-ov5647--imx219).

---

## Potential improvements

- **Better camera:** IMX219 (v2) or IMX477 (HQ) for higher image quality at the same resolution.
- **Pi 4 / 5:** handles 1080p without the CPU bottleneck; may also support DMABuf in `libcamerasrc`.
- **Custom QGC build:** reduce internal jitter buffer for < 200 ms end-to-end.
