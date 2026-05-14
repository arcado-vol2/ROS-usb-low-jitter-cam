#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <time.h>

#include "v4l2_capture.hpp"

// Compute the constant offset (nanoseconds) from CLOCK_MONOTONIC_RAW to
// CLOCK_REALTIME so we can publish ROS-compatible wall-clock stamps derived
// from our CLOCK_MONOTONIC_RAW capture timestamps.
//
// This offset is measured once at startup.  The drift between the two clocks
// over a typical camera session (<1h) is negligible (<1 ms on typical ARM SoCs).
static int64_t g_mono_raw_to_realtime_ns = 0;

static void compute_clock_offset() {
    struct timespec rt{}, raw{};
    clock_gettime(CLOCK_REALTIME,      &rt);
    clock_gettime(CLOCK_MONOTONIC_RAW, &raw);
    g_mono_raw_to_realtime_ns =
        (static_cast<int64_t>(rt.tv_sec)  - static_cast<int64_t>(raw.tv_sec))  * 1000000000LL
      + (static_cast<int64_t>(rt.tv_nsec) - static_cast<int64_t>(raw.tv_nsec));
}

// Convert a CLOCK_MONOTONIC_RAW timespec to ros::Time (wall clock).
static ros::Time mono_raw_to_ros_time(const struct timespec& ts) {
    int64_t ns = static_cast<int64_t>(ts.tv_sec) * 1000000000LL
               + static_cast<int64_t>(ts.tv_nsec)
               + g_mono_raw_to_realtime_ns;
    if (ns < 0) ns = 0;
    return ros::Time(
        static_cast<uint32_t>(ns / 1000000000LL),
        static_cast<uint32_t>(ns % 1000000000LL));
}

// Load integer param with bounds check; returns default_val if missing.
template<typename T>
static T param_clamp(ros::NodeHandle& nh, const std::string& name,
                     T default_val, T lo, T hi) {
    int v = static_cast<int>(default_val);
    nh.param<int>(name, v, static_cast<int>(default_val));
    if (v < static_cast<int>(lo) || v > static_cast<int>(hi)) {
        ROS_WARN("Param %s=%d out of [%d,%d]; using default %d",
                 name.c_str(), v, (int)lo, (int)hi, (int)default_val);
        v = static_cast<int>(default_val);
    }
    return static_cast<T>(v);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "usb_cam_rt");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    compute_clock_offset();

    // ---- Read parameters ----
    CameraConfig cfg;
    pnh.param<std::string>("device",    cfg.device, "/dev/video0");
    pnh.param<int>("width",             reinterpret_cast<int&>(cfg.width),       800);
    pnh.param<int>("height",            reinterpret_cast<int&>(cfg.height),      600);
    pnh.param<int>("num_buffers",       reinterpret_cast<int&>(cfg.num_buffers), 4);
    pnh.param<int>("cpu_core",          cfg.cpu_core,    3);
    pnh.param<int>("rt_priority",       cfg.rt_priority, 90);

    // Camera controls
    cfg.exposure_absolute = param_clamp<int32_t>(pnh, "exposure_absolute",       500,  1,   10000);
    cfg.gain              = param_clamp<int32_t>(pnh, "gain",                     64,  0,    1023);
    cfg.white_balance_temperature = param_clamp<int32_t>(pnh, "white_balance_temperature",
                                                         5000, 2800, 6500);
    cfg.brightness        = param_clamp<int32_t>(pnh, "brightness",                 0, -64,    64);
    cfg.contrast          = param_clamp<int32_t>(pnh, "contrast",                  50,   0,    95);
    cfg.saturation        = param_clamp<int32_t>(pnh, "saturation",               128,   0,   255);
    cfg.sharpness         = param_clamp<int32_t>(pnh, "sharpness",                  0,   0,     7);
    cfg.backlight_compensation = param_clamp<int32_t>(pnh, "backlight_compensation",
                                                      36, 36, 160);
    cfg.focus_absolute    = param_clamp<int32_t>(pnh, "focus_absolute",             0,   0,  1023);
    cfg.power_line_frequency = param_clamp<int32_t>(pnh, "power_line_frequency",   0,   0,     2);

    std::string frame_id;
    pnh.param<std::string>("frame_id", frame_id, "camera");

    // ---- Publisher ----
    // Queue depth 1: subscribers always get the latest frame; never a backlog.
    ros::Publisher img_pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);

    ROS_INFO("usb_cam_rt: device=%s  %ux%u YUYV  rt_priority=%d  cpu_core=%d",
             cfg.device.c_str(), cfg.width, cfg.height,
             cfg.rt_priority, cfg.cpu_core);

    // ---- Capture object ----
    V4L2Capture capture(cfg);

    // Frame callback runs in publish thread (SCHED_OTHER).
    // One memcpy from mmap buffer into ROS message; everything else is zero-copy.
    capture.setFrameCallback([&](const FrameData& frame) {
        sensor_msgs::Image msg;
        msg.header.stamp    = mono_raw_to_ros_time(frame.timestamp);
        msg.header.frame_id = frame_id;
        msg.width           = frame.width;
        msg.height          = frame.height;
        msg.encoding        = "yuv422";         // YUYV packed, 2 bytes/pixel
        msg.is_bigendian    = 0;
        msg.step            = frame.width * 2;  // bytes per row
        msg.data.resize(frame.bytesused);
        std::memcpy(msg.data.data(), frame.data, frame.bytesused);

        // Log a delta between driver timestamp and our raw monotonic stamp.
        // Useful for verifying dequeue latency.
        if (frame.driver_monotonic) {
            double driver_us = static_cast<double>(frame.driver_ts.tv_sec) * 1e6
                             + static_cast<double>(frame.driver_ts.tv_usec);
            double ours_us   = static_cast<double>(frame.timestamp.tv_sec) * 1e6
                             + static_cast<double>(frame.timestamp.tv_nsec) * 1e-3;
            ROS_DEBUG_THROTTLE(5.0, "seq=%u  dequeue delta=%.1f µs  driver_mono=yes",
                               frame.sequence, ours_us - driver_us);
        }

        img_pub.publish(msg);
    });

    // ---- Stats timer (every 5 s) ----
    ros::Timer stats_timer = nh.createTimer(
        ros::Duration(5.0),
        [&](const ros::TimerEvent&) {
            CaptureStats s = capture.getStats();
            ROS_INFO_STREAM(
                "Stats | captured=" << s.frames_captured
                << "  dropped=" << s.frames_dropped
                << "  mean_interval=" << std::fixed
                << static_cast<int>(s.mean_interval_us) << " µs"
                << "  jitter(σ)=" << static_cast<int>(s.jitter_us) << " µs"
                << "  min=" << static_cast<int>(s.min_interval_us) << " µs"
                << "  max=" << static_cast<int>(s.max_interval_us) << " µs");
        });

    try {
        capture.start();
    } catch (const std::exception& e) {
        ROS_FATAL("Failed to start capture: %s", e.what());
        return 1;
    }

    ros::spin();

    capture.stop();
    return 0;
}
