# Video Streaming

H.264 RTP video from a MIPI camera on the Pi to QGroundControl. Each camera appears as a separate MAVLink camera component; each sensor mode is advertised as a stream. QGC starts / stops streams via `MAV_CMD_VIDEO_START_STREAMING`.

Two transports are supported and switchable at runtime via the [`VIDEO_TRANSPORT`](#video-transports) param:

- **RTSP server (default, `VIDEO_TRANSPORT=0`)**: Pi runs `gst-rtsp-server`; QGC pulls from `rtsp://<rover>:8554/stream-N`. Lower latency but smaller QGC-side jitter buffer — motion-induced macroblock corruption shows up more easily on lossy links.
- **UDP push (`VIDEO_TRANSPORT=1`)**: Pi spawns `gst-launch` and pushes RTP/H.264 to `udp://<qgc>:5600`. Higher latency (~200 ms QGC jitter buffer) but absorbs packet loss / reorder more gracefully.

Implementation: [`src/video/`](../../src/video/) + [`src/mavlink/camera_handlers.cpp`](../../src/mavlink/camera_handlers.cpp).

---

## Architecture

```
QGC
  ↓ HEARTBEAT, MAV_CMD_REQUEST_MESSAGE(CAMERA_INFORMATION), …
Pi daemon
  ├── heartbeat per camera component (MAV_COMP_ID_CAMERA..+i)
  ├── CameraDiscovery — parses `libcamera-hello --list-cameras`
  └── VideoBackend (one of):
        ├── RtspBackend → gst-rtsp-server, registers a mount per sensor mode
        │     ↓ pipeline runs while a client is connected
        │   rtsp://<rover>:8554/stream-N  (QGC pulls)
        └── UdpBackend  → fork/exec gst-launch, killpg on stop, watchdog restart
              ↓ pipeline runs after MAV_CMD_VIDEO_START_STREAMING
            udp://<qgc>:5600  (rover pushes RTP/H.264)
```

- Each camera is advertised as its own MAVLink component (`MAV_COMP_ID_CAMERA + i`).
- Each sensor mode from the camera is its own `VIDEO_STREAM_INFORMATION` (stream IDs start at 1).
- `VIDEO_STREAM_INFORMATION.type` is `RTSP` or `RTPUDP` depending on `VIDEO_TRANSPORT`.
- `COMMAND_LONG` is routed by `target_component`: components ≥ 100 go to camera handlers, component 1 is the autopilot.
- `MAV_CMD_VIDEO_START/STOP_STREAMING` are forwarded to the active backend (RTSP no-ops them; UDP spawns/kills the pipeline).
- Backend swaps mid-flight on `VIDEO_TRANSPORT` PARAM_SET — old backend stopped, new one constructed and started in one main-loop iteration.

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
| `VIDEO_TRANSPORT` | 0 | 0 / 1 | 0 = RTSP server, 1 = UDP push (rover→QGC) |
| `VIDEO_BITRATE_1`..`VIDEO_BITRATE_4` | 1.5 / 3 / 5 / 8 Mbps | 25 000 – 25 000 000 | Per-mode H.264 encoder target bitrate (bps); index = sensor mode index |
| `VIDEO_BITRATE` | 5 000 000 | 25 000 – 25 000 000 | Deprecated single-bitrate fallback for modes without a per-mode value |
| `VIDEO_FPS` | 30 | 1 – 60 | Frame rate cap; lowered when the sensor mode's max fps is lower |

Change any of these live from QGC; the active backend is restarted with the new setting. Switching `VIDEO_TRANSPORT` swaps the backend entirely (stop current → construct new → start).

### Video transports

| | RTSP (default) | UDP push |
|---|---|---|
| Direction | QGC pulls (client connects to `rtsp://<rover>:8554/stream-N`) | Rover pushes (sends to `udp://<qgc>:5600`) |
| QGC IP discovery | Not needed (QGC dials in) | Auto-learned from MAVLink source IP — no manual config |
| Latency | ~120-140 ms baseline + QGC ~300 ms decode/jitter buffer | Same baseline + QGC ~200 ms raw-UDP jitter buffer (lower variability) |
| Stability under packet loss | Smaller QGC jitter buffer → motion artifacts surface | Larger jitter buffer absorbs loss/reorder; visibly more stable picture |
| Stream selection (QGC gear dropdown) | Each mode is a distinct mount URL | Each mode is a stream_id; switching kills/respawns the pipeline |
| Idle cost | Zero (pipeline only runs while a client is connected) | Pipeline only runs after `VIDEO_START_STREAMING`; QGC sends this on connect |
| When to prefer | Dev / LAN — lowest latency wins | LTE / lossy links — stable picture wins |

---

## Autostart and QGC-loss behaviour

The stream lifecycle is tied to QGC connection state:

| Event | Action |
|---|---|
| QGC IP first becomes known (boot or first packet) | Stream autostarts at lowest resolution (stream 1 / cam 0 / mode 0 — 640 × 480) |
| QGC silent for > 5 s (`QGC_RECONNECT_GAP_US`) | Stream stops (`[gst] stream stopped`) to save LTE bandwidth |
| QGC reconnects (next packet after silence) | Stream restarts at lowest resolution; banner + autostart `STATUSTEXT` re-sent |
| User starts a specific stream from QGC | Selected resolution overrides autostart |
| `RESET_CAMERA_SETTINGS` (QGC "Reset camera defaults" button) | Stops the running stream (does **not** restart) |

The lowest resolution (640 × 480) is chosen for autostart because it is the cheapest mode on LTE and a safe default; the operator can switch to a higher mode from QGC once connected.

---

## QGC IP detection (UDP-push only)

The UDP-push transport sends RTP/H.264 to `udp://<QGC-IP>:5600`. The daemon learns the QGC IP from the source address of the first MAVLink packet — `recvfrom()` populates it for free, no QGC-side configuration needed. The learned address is logged at first contact (`[udp] pid N started → <ip>:5600 ...`).

The RTSP transport doesn't need this — clients dial in and the rover only needs to know its own listen port.

---

## The GStreamer pipeline

Both transports use the same encoder + payloader; only the sink differs.

**RTSP server (factory pipeline registered per mount):**

```
libcamerasrc !
  video/x-raw,width=W,height=H,framerate=F/1 !
  v4l2h264enc extra-controls="controls,repeat_sequence_header=1,video_bitrate=B" !
  video/x-h264,level=(string)4 !
  h264parse config-interval=-1 !
  rtph264pay config-interval=1 pt=96 mtu=1400 name=pay0
```

**UDP push (forked `gst-launch`):**

```bash
gst-launch-1.0 libcamerasrc ! \
  video/x-raw,width=W,height=H,framerate=F/1 ! \
  v4l2h264enc extra-controls="controls,repeat_sequence_header=1,video_bitrate=B" ! \
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
