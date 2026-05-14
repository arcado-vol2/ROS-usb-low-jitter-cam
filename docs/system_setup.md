# System Setup – Low-Latency USB Camera on Orange Pi 5 Max (Ubuntu 20.04)

Every command below must be run on the target board.  
Commands that persist across reboots are marked **[PERSISTENT]**.

---

## 1. Install ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" \
    > /etc/apt/sources.list.d/ros-latest.list'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-ros-base python3-catkin-tools python3-rosdep
sudo rosdep init && rosdep update
```

---

## 2. Build the package

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
# Clone or copy the package directory here as usb_cam_rt/
catkin_init_workspace
cd ~/catkin_ws
catkin build usb_cam_rt
source devel/setup.bash
```

---

## 3. Real-time scheduling permissions  **[PERSISTENT]**

The capture thread requests `SCHED_FIFO`.  Either run the node as root (not
recommended in production) or grant `RLIMIT_RTPRIO` to the user:

```bash
# Replace 'orangepi' with the actual username.
sudo bash -c 'cat >> /etc/security/limits.conf << EOF
orangepi  -  rtprio   99
orangepi  -  memlock  unlimited
EOF'
# Re-login for limits to apply.
```

Verify after re-login:

```bash
ulimit -r   # should print 99
ulimit -l   # should print unlimited
```

---

## 4. Isolate a CPU core for the capture thread  **[PERSISTENT]**

The Orange Pi 5 Max has 8 cores (4× Cortex-A55 + 4× Cortex-A76).
Core 3 is a Cortex-A55 cluster core that is largely idle; core 7 (A76) gives
better single-thread performance if you prefer.

Edit the U-Boot / extlinux boot arguments:

```bash
sudo nano /boot/extlinux/extlinux.conf
```

Append to the `APPEND` line (add after `root=...`):

```
isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll
```

Full example:
```
APPEND root=/dev/mmcblk0p2 rootfstype=ext4 ... isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll
```

Reboot and verify:

```bash
cat /sys/devices/system/cpu/isolated   # should print: 3
```

Update `cpu_core` in `config/camera_params.yaml` to match.

---

## 5. CPU frequency governor  **[PERSISTENT]**

Prevent the scheduler from throttling the isolated core:

```bash
# Apply immediately:
echo performance | sudo tee /sys/devices/system/cpu/cpu3/cpufreq/scaling_governor

# Make persistent via rc.local or systemd service:
sudo bash -c 'cat > /etc/systemd/system/cpu-governor.service << EOF
[Unit]
Description=Set CPU governor to performance
After=multi-user.target

[Service]
Type=oneshot
ExecStart=/bin/bash -c "echo performance | tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor"
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF'
sudo systemctl enable --now cpu-governor.service
```

---

## 6. USB autosuspend – disable for the camera  **[PERSISTENT]**

```bash
# Find the camera's USB path (look for "Global Shutter" in lsusb -v or dmesg):
USBPATH=$(ls /sys/bus/usb/devices/ | xargs -I{} sh -c \
    'cat /sys/bus/usb/devices/{}/product 2>/dev/null | grep -q "Global" && echo {}')
echo "Camera USB path: $USBPATH"

# Disable autosuspend immediately:
echo -1 | sudo tee /sys/bus/usb/devices/${USBPATH}/power/autosuspend_delay_ms
echo on  | sudo tee /sys/bus/usb/devices/${USBPATH}/power/control
```

Persistent udev rule:

```bash
# Get vendor/product IDs:
lsusb | grep -i "global"
# Example output: Bus 001 Device 003: ID 1234:5678 ...

sudo bash -c 'cat > /etc/udev/rules.d/99-usb-camera-rt.rules << EOF
# EPL USBGS1200P01 Global Shutter Camera – disable USB autosuspend
ACTION=="add", SUBSYSTEM=="usb", ATTR{idVendor}=="XXXX", ATTR{idProduct}=="YYYY", \
    ATTR{power/autosuspend_delay_ms}="-1", ATTR{power/control}="on"

# Allow the usb_cam_rt user to access the video device without sudo
ACTION=="add", SUBSYSTEM=="video4linux", KERNEL=="video[0-9]*", \
    GROUP="video", MODE="0660"
EOF'

sudo udevadm control --reload-rules && sudo udevadm trigger
```

Replace `XXXX`/`YYYY` with actual vendor/product IDs from `lsusb`.

Add your user to the `video` group:

```bash
sudo usermod -aG video orangepi
# Re-login to apply.
```

---

## 7. USB latency timer

```bash
# Set USB device polling interval to 1 ms (minimum):
USBPATH=...   # same as step 6
echo 1 | sudo tee /sys/bus/usb/devices/${USBPATH}/latency_timer 2>/dev/null || true
```

---

## 8. IRQ affinity – steer USB interrupts away from isolated core

```bash
# List IRQs related to USB:
grep -i usb /proc/interrupts | awk '{print $1}' | tr -d ':'

# For each USB IRQ number N, steer it away from core 3 (bit mask without core 3):
# 8-core mask without core 3: binary 11110111 = 0xf7
for IRQ in $(grep -i usb /proc/interrupts | awk '{print $1}' | tr -d ':'); do
    echo f7 | sudo tee /proc/irq/${IRQ}/smp_affinity 2>/dev/null || true
done
```

Persistent via `/etc/rc.local` or a systemd ExecStart script.

---

## 9. Disable kernel timer coalescing on isolated core

```bash
# nohz_full=3 in step 4 already achieves this; verify with:
cat /sys/devices/system/cpu/cpu3/cpuidle/state*/name
# Ideally only state "WFI" should appear for the isolated core under load.
```

---

## 10. Lock memory limits for the process

`mlockall(MCL_CURRENT | MCL_FUTURE)` is called inside the node (requires
`memlock` unlimited, set in step 3).  This prevents page faults during the
realtime capture loop.

---

## 11. Verify the setup before running

```bash
# Check isolated CPUs:
cat /sys/devices/system/cpu/isolated

# Check RT limits:
ulimit -r && ulimit -l

# Check camera device:
v4l2-ctl --device=/dev/video0 --list-formats-ext | grep -A3 YUYV

# Check that 800x600 YUYV is listed:
v4l2-ctl --device=/dev/video0 --list-framesizes=YUYV

# Confirm all controls are writable:
v4l2-ctl --device=/dev/video0 --list-ctrls
```

---

## 12. Run the node

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch usb_cam_rt camera.launch
```

To override parameters at launch time:

```bash
roslaunch usb_cam_rt camera.launch device:=/dev/video2 cpu_core:=7
```
