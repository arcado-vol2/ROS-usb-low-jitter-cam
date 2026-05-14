# Measurement Guide – Jitter, Latency, and Frame Drops

All measurements assume the node is running and publishing to `/usb_cam_rt/image_raw`.

---

## 1. Built-in statistics (ROS log)

The node prints a stats line every 5 seconds:

```
[INFO] Stats | captured=150  dropped=0  mean_interval=33333 µs  jitter(σ)=42 µs  min=33291 µs  max=33421 µs
```

| Field           | Meaning |
|-----------------|---------|
| `mean_interval` | Mean time between consecutive frame dequeues (µs). Expected ≈ 33 333 µs at 30 fps |
| `jitter(σ)`     | Standard deviation of inter-frame intervals (µs). Target: < 200 µs |
| `min` / `max`   | Extreme inter-frame intervals seen so far |
| `dropped`       | Frames the RT thread dropped because the publish thread was busy |

---

## 2. Per-frame dequeue delta (debug log)

Enable `ROS_LOG_LEVEL=DEBUG` to see per-frame delta between the driver-reported
monotonic timestamp and our `CLOCK_MONOTONIC_RAW` dequeue timestamp:

```bash
ROS_LOG_LEVEL=DEBUG roslaunch usb_cam_rt camera.launch 2>&1 | grep "dequeue delta"
```

Expected delta: 0–500 µs (time the frame spends in driver buffers + OS scheduler
wake-up jitter).  Values > 2 ms indicate system overload or a non-RT kernel.

---

## 3. Timestamp jitter via rostopic

```bash
# Print message timestamps and compute interval jitter with Python:
rostopic echo /usb_cam_rt/image_raw/header/stamp | \
python3 - << 'EOF'
import sys, re, statistics

stamps = []
for line in sys.stdin:
    m = re.search(r'secs:\s*(\d+)', line)
    if m:
        secs = int(m.group(1))
    m = re.search(r'nsecs:\s*(\d+)', line)
    if m:
        ns = secs * 1e9 + int(m.group(1))
        stamps.append(ns)
        if len(stamps) > 1:
            intervals = [stamps[i] - stamps[i-1] for i in range(1, len(stamps))]
            print(f"n={len(stamps)-1}  mean={statistics.mean(intervals)/1e3:.1f} µs"
                  f"  σ={statistics.stdev(intervals)/1e3:.1f} µs"
                  f"  min={min(intervals)/1e3:.1f} µs"
                  f"  max={max(intervals)/1e3:.1f} µs", flush=True)
EOF
```

Collect at least 300 frames (10 s) for meaningful statistics.

---

## 4. Frame drop detection

```bash
# Monitor the sequence field; gaps indicate driver-level drops:
rostopic echo /usb_cam_rt/image_raw/header/seq | \
awk 'BEGIN{prev=-1} /^[0-9]/{
    seq=$1; if(prev>=0 && seq!=prev+1) print "DROP: seq jumped from "prev" to "seq;
    prev=seq
}'
```

Note: the node also logs a `WARN` line for each frame dropped at the user-space
level (publish thread too slow), separately from driver-level drops.

---

## 5. End-to-end latency with a hardware trigger or LED flash

For absolute latency measurement (capture moment → ROS topic):

1. Connect an LED to a GPIO output.
2. Flash the LED and record `ros::Time::now()` at the flash event.
3. Subscribe to `/usb_cam_rt/image_raw` and detect the frame where the LED
   appears (brightness threshold in a ROI).
4. Latency = `msg.header.stamp` – GPIO flash time.

Expected: 1–2 frame periods (33–66 ms) for USB bulk transfer + USB host
scheduling + user-space dequeue.

---

## 6. Verify auto controls are disabled

```bash
# After the node starts, query controls; values must match camera_params.yaml:
v4l2-ctl --device=/dev/video0 --get-ctrl=\
exposure_auto,exposure_absolute,gain,white_balance_temperature_auto,\
white_balance_temperature,focus_auto,focus_absolute
```

Expected output (with default config):
```
exposure_auto: 1
exposure_absolute: 500
gain: 64
white_balance_temperature_auto: 0
white_balance_temperature: 5000
focus_auto: 0
focus_absolute: 0
```

Shine a torch at the camera and re-check: values must not change.

---

## 7. Scheduling statistics

```bash
# Check that the capture thread is running at SCHED_FIFO:
ps -eLo pid,tid,cls,pri,psr,comm | grep usb_cam

# Expected: cls=FF (FIFO), pri=90, psr=3 (CPU core 3)
```

```bash
# Check scheduler migration events on the isolated core:
perf stat -e sched:sched_migrate_task -C 3 sleep 5
# Expected count: 0 (nothing migrates onto the isolated core)
```

---

## 8. Buffer count experiments

Change `num_buffers` in `camera_params.yaml` and restart the node:

| `num_buffers` | Expected behaviour |
|---|---|
| 2 | Minimum; any scheduling glitch > 1 frame period causes a drop |
| 4 | Recommended; 2 spare buffers absorb brief OS jitter |
| 8 | High latency (≈ 8 frames buffered); not recommended for low-latency use |

Record `max_interval_us` and `dropped` from the stats log for each setting.

---

## 9. Recording a jitter log to file

```bash
# Redirect node output to a file and extract stats lines:
roslaunch usb_cam_rt camera.launch 2>&1 | tee /tmp/camera_run.log &
sleep 60
grep "Stats |" /tmp/camera_run.log > /tmp/jitter_stats.txt
cat /tmp/jitter_stats.txt
```
