#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <functional>
#include <atomic>
#include <pthread.h>
#include <linux/videodev2.h>
#include <time.h>

// All fixed camera control values for EPL USBGS1200P01 global shutter camera.
// Defaults mirror camera_params.yaml; override via ROS params or YAML.
struct CameraConfig {
    std::string device           = "/dev/video0";
    uint32_t    width            = 800;
    uint32_t    height           = 600;
    uint32_t    bytesperline     = 0;   // set by setFormat() from driver; may include padding
    uint32_t    num_buffers      = 4;   // >=2; 4 gives 2 spare queued while 1 is processed
    int         cpu_core         = 3;
    int         rt_priority      = 90;

    // V4L2 controls – disable all auto adjustments, lock concrete values
    int32_t     exposure_absolute          = 500;    // 1-10000 (units: 100 µs)
    int32_t     gain                       = 64;     // 0-1023
    int32_t     white_balance_temperature  = 5000;   // 2800-6500 K
    int32_t     brightness                 = 0;      // -64 to 64
    int32_t     contrast                   = 50;     // 0-95
    int32_t     saturation                 = 128;    // 0-255
    int32_t     sharpness                  = 0;      // 0-7
    int32_t     backlight_compensation     = 36;     // 36-160 (minimum = off)
    int32_t     focus_absolute             = 0;      // 0-1023 (fixed focus)
    int32_t     power_line_frequency       = 0;      // 0=disabled,1=50Hz,2=60Hz
};

// Data handed to the user callback – points directly into the kernel mmap buffer.
// The pointer is valid only for the duration of the callback; do not cache it.
struct FrameData {
    const uint8_t*  data;           // mmap pointer into kernel V4L2 buffer (YUYV)
    size_t          bytesused;      // actual bytes in this frame
    uint32_t        width;
    uint32_t        height;
    uint32_t        bytesperline;   // driver row stride in bytes (may exceed width*2)
    struct timespec timestamp;      // CLOCK_MONOTONIC_RAW at exact dequeue moment
    uint32_t        sequence;       // monotonically increasing driver sequence number
    bool            driver_monotonic; // true if driver also reported monotonic ts
    struct timeval  driver_ts;      // driver-reported timestamp (for delta logging)
};

struct CaptureStats {
    uint64_t frames_captured     = 0;
    uint64_t frames_dropped      = 0;
    double   mean_interval_us    = 0.0;   // mean inter-frame interval
    double   jitter_us           = 0.0;   // std-dev of inter-frame interval
    double   max_interval_us     = 0.0;
    double   min_interval_us     = 1e9;
};

// V4L2Capture: zero-copy mmap capture with dedicated SCHED_FIFO RT thread.
//
// Thread model:
//   capture thread (SCHED_FIFO, pinned CPU) – epoll + VIDIOC_DQBUF + timestamp
//   publish thread (SCHED_OTHER)            – callback + VIDIOC_QBUF
//
// The capture thread never blocks on the publish thread: if the publish thread
// hasn't returned the previous buffer in time, the new frame overwrites it
// (drop-on-overflow).  The publish thread reads directly from the mmap buffer
// (zero copy) before re-queuing it to the driver.
class V4L2Capture {
public:
    using FrameCallback = std::function<void(const FrameData&)>;

    explicit V4L2Capture(const CameraConfig& config);
    ~V4L2Capture();

    // Must be called before start().
    void setFrameCallback(FrameCallback cb);

    void start();
    void stop();
    bool isRunning() const { return running_.load(std::memory_order_relaxed); }

    CaptureStats getStats() const;

private:
    void openDevice();
    void setControls();
    void setFormat();
    void initMmap();
    void startStreaming();
    void stopStreaming();
    void unmapBuffers();
    void closeDevice();
    void setControl(uint32_t cid, int32_t value, const char* name);

    static void* captureThreadEntry(void* arg);
    void captureLoop();

    static void* publishThreadEntry(void* arg);
    void publishLoop();

    CameraConfig      config_;
    int               fd_       = -1;
    int               epoll_fd_ = -1;
    std::atomic<bool> running_  {false};
    FrameCallback     frame_cb_;

    struct MmapBuffer {
        uint8_t* start   = nullptr;
        size_t   length  = 0;
    };
    std::vector<MmapBuffer> mmap_bufs_;

    // Single-slot hand-off: RT capture thread -> publish thread.
    // Protected by frame_mutex_ / frame_cond_.
    struct SharedFrame {
        bool            valid     = false;
        uint32_t        buf_index = 0;
        uint32_t        bytesused = 0;
        struct timespec timestamp {};
        uint32_t        sequence  = 0;
        bool            driver_monotonic = false;
        struct timeval  driver_ts {};
    };

    pthread_mutex_t frame_mutex_ = PTHREAD_MUTEX_INITIALIZER;
    pthread_cond_t  frame_cond_  = PTHREAD_COND_INITIALIZER;
    SharedFrame     shared_frame_;

    pthread_t capture_thread_;
    pthread_t publish_thread_;

    // Stats – protected by stats_mutex_
    mutable pthread_mutex_t stats_mutex_ = PTHREAD_MUTEX_INITIALIZER;
    CaptureStats     stats_;
    double           mean_interval_us_ = 0.0;
    double           M2_interval_      = 0.0;     // Welford accumulator
    struct timespec  last_frame_ts_    {};
    bool             first_frame_      = true;
};
