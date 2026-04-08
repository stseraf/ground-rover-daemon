# Video Streaming: RPi Zero 2W to QGC (Windows)

## Hardware

- **Camera**: OV5647 (RPi Camera v1), 5MP, 10-bit Bayer GBRG
- **SBC**: Raspberry Pi Zero 2W (4-core ARM, 416MB RAM total)
- **OS**: Debian Trixie (13), kernel 6.12.75
- **HW Encoder**: bcm2835-codec (`v4l2h264enc` on `/dev/video11`), H.264 baseline/main/high up to level 5.1
- **Network (wired)**: RPi eth hat -> UTP -> MikroTik cAP -> UTP -> MikroTik hAP -> UTP -> switch -> Win laptop
- **Network (WiFi)**: RPi onboard WiFi (BCM43438, 2.4GHz only, single-stream 802.11n) -> MikroTik cAP (CAPsMAN managed) -> MikroTik hAP -> switch -> client
- **Wired link speed**: 100 Mbps (RPi eth hat limit), ~1-2ms RTT, 0% packet loss
- **Receiver**: QGroundControl on Windows, listens on UDP port 5600 for RTP H.264

## Camera Sensor Modes

| Resolution | Max FPS | Sensor Crop | FOV | Notes |
|-----------|---------|-------------|-----|-------|
| 640x480 | 58.92 | (16,0)/2560x1920 | Full 4:3 | Binned 4x |
| **1296x972** | **46.34** | **(0,0)/2592x1944** | **Full 4:3** | **Binned 2x, best for rover** |
| 1920x1080 | 32.81 | (348,434)/1928x1080 | Center crop 16:9 | Loses ~44% of sensor area |
| 2592x1944 | 15.63 | (0,0)/2592x1944 | Full 4:3 | Full resolution, too slow |

**Key insight**: 1296x972 uses the full sensor with 2x binning, giving the widest field of view. 1920x1080 center-crops a 16:9 window from the sensor, significantly narrowing the FOV. For a rover, 1296x972 provides much better situational awareness.

## GPU Memory Configuration

Default `gpu_mem=64` is not enough for the HW encoder at higher resolutions. Set in `/boot/firmware/config.txt`:

```
gpu_mem=128
```

This leaves 288MB for ARM (enough for headless streaming). Without this, the encoder fails with `"Failed to process frame. Maybe be due to not enough memory or failing driver"` and `"bcm2835_codec_start_streaming: Failed enabling i/p port, ret -3"`.

**Warning**: Failed encode attempts (especially at 1080p) can leave the bcm2835-codec driver in a broken state where even 640x480 fails. Fix by either rebooting or reloading the module:

```bash
sudo modprobe -r bcm2835_codec && sudo modprobe bcm2835_codec
```

## DMABuf / Zero-Copy

The `v4l2h264enc` encoder accepts DMABuf input (`video/x-raw(memory:DMABuf)`), which would allow zero-copy from camera to encoder. However, **`libcamerasrc` (GStreamer 1.26 on Debian Trixie) does not export DMABuf** — it only outputs plain `video/x-raw` (system memory). This means every frame is CPU-copied into the encoder.

- `libcamerasrc` src pad caps: `video/x-raw`, `image/jpeg`, `video/x-bayer` only
- `v4l2h264enc` sink accepts: `video/x-raw(memory:DMABuf)` and `video/x-raw`
- Negotiated: `video/x-raw` format NV21 with MMAP buffer pool (mode 2) + copy
- Forcing `output-io-mode=dmabuf-import` on the encoder fails because libcamerasrc can't provide DMABuf buffers

**`rpicam-vid` uses DMABuf internally** (camera -> ISP -> encoder all zero-copy via MMAL), which is why it uses significantly less CPU.

## CPU Usage Comparison

| Pipeline | Resolution | CPU Usage | Notes |
|----------|-----------|-----------|-------|
| `libcamerasrc` + `v4l2h264enc` | 640x480 | ~25% | Comfortable |
| `libcamerasrc` + `v4l2h264enc` | 1296x972 | ~50% | Good for rover use |
| `libcamerasrc` + `v4l2h264enc` | 1920x1080 | ~100% (1 core maxed) | Tight, may drop frames |
| `rpicam-vid` + GStreamer RTP | 1920x1080 | ~47% (33% rpicam + 14% gst) | DMABuf zero-copy |
| Capture only (no encoder) | any | ~0% | ISP debayering is HW accelerated |

The CPU load comes from the memory copy between `libcamerasrc` and `v4l2h264enc`, not from the encoding or ISP processing — both are hardware accelerated.

## Encoder Settings

Available v4l2 controls (`v4l2-ctl -d /dev/video11 --list-ctrls`):

| Control | Range | Default | Notes |
|---------|-------|---------|-------|
| `video_bitrate` | 25000-25000000 | 10000000 | In bits/sec |
| `video_bitrate_mode` | 0-1 | 0 (VBR) | 1 = CBR |
| `video_gop_size` | 0-2147483647 | 60 | |
| `h264_i_frame_period` | 0-2147483647 | 60 | Keyframe interval in frames |
| `h264_profile` | 0-4 | 4 (High) | 0=baseline, 4=high |
| `h264_level` | 0-15 | 11 (4) | |
| `h264_minimum_qp_value` | 0-51 | 20 | |
| `h264_maximum_qp_value` | 0-51 | 51 | |
| `repeat_sequence_header` | bool | false | SPS/PPS with every I-frame |
| `video_b_frames` | 0 | 0 | B-frames not supported |

## Latency Analysis

### Measured

- **GStreamer receive (zero jitter buffer)**: ~120-140ms end-to-end
- **QGC**: ~450ms end-to-end
- **QGC overhead**: ~300ms (internal `rtpjitterbuffer latency=200` + decode/render)
- **Network RTT**: ~2ms (1ms one-way)

### Theoretical Breakdown

| Stage | Estimated |
|-------|-----------|
| Camera capture (1 frame @ 30fps) | ~33ms |
| HW encode | ~10-15ms |
| Network | ~1-2ms |
| Decode + render | ~15-30ms |
| **Minimum total** | **~60-80ms** |

QGC adds ~300ms on top due to its internal jitter buffer and processing.

## Recommended Pipelines

### Option A: Pure GStreamer (1296x972, best FOV)

Best for rover use — full sensor FOV, ~50% CPU, works with QGC directly.

```bash
gst-launch-1.0 libcamerasrc ! \
  video/x-raw,width=1296,height=972,framerate=30/1 ! \
  v4l2h264enc extra-controls="controls,repeat_sequence_header=1,video_bitrate=5000000" ! \
  'video/x-h264,level=(string)4' ! \
  h264parse config-interval=-1 ! \
  rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=192.168.50.59 port=5600 sync=false
```

### Option B: rpicam-vid + GStreamer RTP (1080p, lower CPU)

Use when 1080p 16:9 is needed. DMABuf zero-copy internally, ~47% CPU.

```bash
rpicam-vid -t 0 --width 1920 --height 1080 --framerate 30 \
  --bitrate 8000000 --profile baseline --level 4 \
  --inline --flush --intra 30 -n -o - | \
gst-launch-1.0 fdsrc ! h264parse ! \
  rtph264pay config-interval=1 pt=96 mtu=1400 ! \
  udpsink host=192.168.50.59 port=5600 sync=false
```

### Option C: Low-latency GStreamer receive on Windows

Bypasses QGC's ~300ms jitter buffer. Use for latency-critical operation.

```
gst-launch-1.0 udpsrc port=5600 caps="application/x-rtp,encoding-name=H264,payload=96" ! \
  rtpjitterbuffer latency=0 ! rtph264depay ! avdec_h264 ! d3dvideosink sync=false
```

Requires GStreamer installed on Windows (installed at `C:\gstreamer\1.0\x86_64\`).

## Pipeline Parameter Reference

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `repeat_sequence_header=1` | bool | SPS/PPS with every I-frame for stream join |
| `video_bitrate=5000000` | 5 Mbps | Good quality at 1296x972 on wired link |
| `config-interval=-1` (h264parse) | | Send SPS/PPS with every I-frame |
| `config-interval=1` (rtph264pay) | | Include SPS/PPS in RTP for stream recovery |
| `mtu=1400` | bytes | Fits standard MTU, avoids IP fragmentation |
| `sync=false` | | No clock sync on output, reduces latency |
| `pt=96` | | RTP payload type for H.264 |
| `--inline` (rpicam-vid) | | SPS/PPS with every I-frame |
| `--flush` (rpicam-vid) | | Push data immediately, no output buffering |
| `--intra 30` (rpicam-vid) | frames | Keyframe every 1 second at 30fps |
| `-n` (rpicam-vid) | | No preview window |

## Potential Improvements

- **Upgrade camera**: IMX219 (v2) or IMX477 (HQ) for better image quality at same resolutions
- **Upgrade SBC**: Pi 4/5 handles 1080p ISP without CPU bottleneck and may support DMABuf in libcamerasrc
- **Build GStreamer from source**: Newer libcamera GStreamer plugin may support DMABuf export
- **Custom QGC build**: Reduce internal jitter buffer for lower latency
- **systemd service**: Auto-start streaming on boot (not yet configured)
- **USB WiFi adapter with 5GHz**: Would bypass the BCM43438's 2.4GHz single-stream limitation, enabling higher bitrate streaming over WiFi

## Network Test Data

### Wired (Ethernet hat)

iperf3 UDP test (4 Mbps target):
- Sustained 4.00 Mbits/sec with 0% packet loss
- Jitter: 0.1-2.0ms
- Link can handle much more (100 Mbps wired)

### WiFi (BCM43438, 2.4GHz 802.11n)

RPi WiFi chip: Broadcom BCM43438, 2.4GHz only, single spatial stream, max link rate 54-72 Mbps (802.11n HT20).

**Home network setup**: 2x MikroTik cAP + 1x MikroTik hAP, all managed by CAPsMAN on hAP. SSID: S_HOME.

**Problems found and fixed (2026-04-07)**:

1. **Co-channel interference**: Both 2.4GHz APs (cap1-1, cap2-1) on the same channel (Ch 1 / 2412 MHz).
   Fix: Separated channels — cap1-1 on Ch 1 (2412 MHz), cap2-1 on Ch 11 (2462 MHz). Ch 6 was tried first but had neighbor AP interference (SC9S at -65 dBm).

2. **WiFi power save**: RPi's BCM43438 had power save enabled by default, causing periodic 50-90ms latency spikes.
   Fix: `sudo iw dev wlan0 set power_save off` + `nmcli con modify S_HOME 802-11-wireless.powersave 2` (persistent).

3. **Uncontrolled roaming**: RPi would associate with distant cap1-1 instead of nearby cap2-1.
   Fix: BSSID-pinned to cap2-1 via `nmcli con modify S_HOME 802-11-wireless.bssid 04:F4:1C:4B:F0:77`.

4. **CAPsMAN access-list too aggressive**: -65 dBm threshold caused a kick-reconnect loop every 60s.
   Fix: Relaxed to -75 dBm with 30s grace, then disabled entirely (not needed with BSSID pinning).

5. **CAPsMAN WPA1+WPA2 mixed mode**: Changed to WPA2-only (`security.authentication-types=wpa2-psk`).

6. **brcmfmac firmware bug (BCM43430 firmware 7.45.96)**: The firmware has an internal ~60s timer that triggers a locally-generated deauthentication (reason=3), resetting the radio and regulatory domain. This causes `completed -> disconnected` every 60 seconds regardless of AP configuration. Root cause: firmware's `ccode=ALL` conflicts with AP's country IE (UA), triggering periodic `CTRL-EVENT-REGDOM-CHANGE` cycles. Changing NVRAM `ccode=UA` did not fix it — the firmware ignores the NVRAM parameter for this chip variant (43436s).
   Mitigation: Limited wpa_supplicant scan to channel 11 only (`wpa_cli set_network 0 scan_freq 2462`), reducing reconnect time from 5-6 seconds to sub-second. Persisted via NM dispatcher script at `/etc/NetworkManager/dispatcher.d/99-scan-freq`.

7. **cAP2 default config cleanup**: Removed leftover DHCP server, DNS resolver, UPnP, NAT masquerade, stale DNS records, disabled `detect-internet`.

**iperf3 UDP results after fix** (RPi 192.168.50.154 -> Mac 192.168.50.46, 30s each, 1316-byte packets simulating H264 RTP):

| Target bitrate | Received bitrate | Jitter | Packet loss | Lost/Total | Verdict |
|---------------|-----------------|--------|-------------|------------|---------|
| 5 Mbps | 4.96 Mbps | 0.26 ms | 0.81% | 116/14,249 | Good for 1080p streaming |
| 10 Mbps | 9.89 Mbps | 0.80 ms | 1.02% | 291/28,497 | Borderline, occasional artifacts |
| 25 Mbps | 22.32 Mbps | 0.72 ms | 10.38% | 7,392/71,240 | Unusable, exceeds radio capacity |

**WiFi throughput ceiling**: Effective max ~15-20 Mbps before packet loss becomes unacceptable. For 1080p H264 at 5-8 Mbps the link is comfortably within safe margins.

**RPi WiFi connection details**:
- Connected AP: cap2-1 (BSSID 04:F4:1C:4B:F0:77), channel 11 (2462 MHz)
- Signal: 90-94/100 (-25 to -30 dBm)
- Link speed: 54 Mbps (HT20 negotiated)
- RPi MAC: 2C:CF:67:91:18:41
- Power save: off
- Firmware roaming: off (roamoff=1)
- Scan freq: 2462 only (fast reconnect after firmware disconnect)

**Known limitation**: brcmfmac firmware disconnects every ~60s (reason=3, locally_generated). With scan_freq limited to ch11, reconnect is sub-second — ping shows 0% loss, with occasional ~80ms spike at reconnect moment. Acceptable for 5 Mbps H264 streaming.
