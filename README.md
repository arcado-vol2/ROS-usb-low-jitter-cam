# usb_cam_rt – Low-Latency USB Camera Node for ROS Noetic

A minimal, production-quality ROS Noetic camera node designed for the
**EPL USBGS1200P01** global shutter USB camera on **Orange Pi 5 Max** (Ubuntu 20.04).
All design choices prioritise minimum and stable capture latency over convenience.

---

## Key design decisions

| Requirement | Implementation |
|---|---|
| 800×600 YUYV capture | `VIDIOC_S_FMT` with `V4L2_PIX_FMT_YUYV`; error if driver rejects |
| No FFmpeg / MJPEG | Pure V4L2 + libc; no external decoder |
| All auto controls off | `exposure_auto=1`, `focus_auto=0`, `auto_white_balance=0` set before streaming |
| CLOCK_MONOTONIC_RAW | `clock_gettime(CLOCK_MONOTONIC_RAW)` immediately after `VIDIOC_DQBUF` |
| Minimum buffering | `VIDIOC_REQBUFS` with configurable count (default 4); 2 queued in driver at all times |
| Zero-copy pipeline | Kernel mmap buffers; publish thread reads directly from mmap before re-queuing |
| RT capture thread | `SCHED_FIFO` priority 90, `pthread_setaffinity_np` to isolated core |
| Non-blocking I/O | `O_NONBLOCK` + `epoll` in capture thread; no blocking syscalls |
| Drop-on-overflow | If publish thread is slow, capture thread drops the un-consumed frame |
| ROS image output | `sensor_msgs/Image` encoding `yuv422`; one `memcpy` from mmap to ROS message |

---

## Package layout

```
usb_cam_rt/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── camera_params.yaml      # all tunable parameters
├── launch/
│   └── camera.launch
├── src/
│   ├── v4l2_capture.hpp        # V4L2Capture class declaration
│   ├── v4l2_capture.cpp        # capture + publish threads, controls, mmap
│   └── usb_cam_node.cpp        # ROS node, param loading, publisher
└── docs/
    ├── system_setup.md         # kernel, udev, isolcpus, RT limits setup
    └── measurement_guide.md    # how to measure jitter, latency, drops
```

---

## Quick start

```bash
# 1. System preparation (one-time, see docs/system_setup.md for details)
sudo usermod -aG video $USER
# Add RT limits to /etc/security/limits.conf, add isolcpus=3 to boot args, reboot.

# 2. Build
mkdir -p ~/catkin_ws/src && cp -r usb_cam_rt ~/catkin_ws/src/
cd ~/catkin_ws && catkin build usb_cam_rt
source devel/setup.bash

# 3. Run
roslaunch usb_cam_rt camera.launch

# 4. View
rosrun image_view image_view image:=/usb_cam_rt/image_raw
```

---

## Parameters

All parameters live in `config/camera_params.yaml` and can be overridden at launch:

```bash
roslaunch usb_cam_rt camera.launch device:=/dev/video2 cpu_core:=7
```

| Parameter | Default | Range | Notes |
|---|---|---|---|
| `device` | `/dev/video0` | – | V4L2 device path |
| `width` | `800` | – | YUYV capture width |
| `height` | `600` | – | YUYV capture height |
| `num_buffers` | `4` | ≥2 | Fewer = less latency, more risk of drops |
| `cpu_core` | `3` | 0–7 | Must match `isolcpus=` boot arg |
| `rt_priority` | `90` | 1–99 | SCHED_FIFO priority |
| `exposure_absolute` | `500` | 1–10000 | 100 µs units |
| `gain` | `64` | 0–1023 | |
| `white_balance_temperature` | `5000` | 2800–6500 | Kelvin |
| `focus_absolute` | `0` | 0–1023 | Fixed focus position |
| `brightness` | `0` | -64–64 | |
| `contrast` | `50` | 0–95 | |
| `saturation` | `128` | 0–255 | |
| `sharpness` | `0` | 0–7 | |
| `backlight_compensation` | `36` | 36–160 | 36 = minimum/off |
| `power_line_frequency` | `0` | 0–2 | 0=disabled, 1=50Hz, 2=60Hz |

---

## Published topics

| Topic | Type | Notes |
|---|---|---|
| `image_raw` | `sensor_msgs/Image` | `yuv422` encoding (YUYV packed), header.stamp from `CLOCK_MONOTONIC_RAW` converted to wall time |

---

## Timestamp details

The node calls `clock_gettime(CLOCK_MONOTONIC_RAW, &ts)` immediately after
`VIDIOC_DQBUF` returns in the SCHED_FIFO capture thread.  This is the lowest-
jitter timestamp achievable in user-space.

The one-time offset between `CLOCK_MONOTONIC_RAW` and `CLOCK_REALTIME` is
computed at startup and applied to all published timestamps so that
`header.stamp` remains compatible with `ros::Time` (wall clock).

Enable `ROS_LOG_LEVEL=DEBUG` to see per-frame dequeue delta (our timestamp –
driver timestamp).

---

## Measuring jitter and latency

See **[docs/measurement_guide.md](docs/measurement_guide.md)** for:
- Reading built-in stats from the ROS log
- Computing inter-frame jitter with `rostopic echo`
- Detecting frame drops via sequence gaps
- Measuring end-to-end latency with an LED trigger
- Buffer count experiments

---

## System setup

See **[docs/system_setup.md](docs/system_setup.md)** for step-by-step instructions:
- RT scheduling permissions (`RLIMIT_RTPRIO`, `RLIMIT_MEMLOCK`)
- `isolcpus=3` kernel boot argument
- CPU frequency governor (`performance`)
- USB autosuspend disable (udev rule)
- IRQ affinity (steer USB IRQs away from isolated core)

---

## Changing the exposure / gain at runtime

While the node locks controls on startup, you can update them from the shell
(they will be re-applied on the next node restart, or you can send them live
via `v4l2-ctl`):

```bash
# Increase exposure (brighter / longer integration):
v4l2-ctl --device=/dev/video0 --set-ctrl=exposure_absolute=2000

# Reduce gain (less noise):
v4l2-ctl --device=/dev/video0 --set-ctrl=gain=32
```

For permanent changes, edit `config/camera_params.yaml`.
