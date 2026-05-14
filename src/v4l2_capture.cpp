#include "v4l2_capture.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/epoll.h>
#include <linux/videodev2.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <sched.h>
#include <pthread.h>
#include <ros/ros.h>

// ioctl wrapper that retries on EINTR.
static int xioctl(int fd, unsigned long request, void* arg) {
    int r;
    do { r = ::ioctl(fd, request, arg); } while (r == -1 && errno == EINTR);
    return r;
}

static inline double ts_to_us(const struct timespec& ts) {
    return static_cast<double>(ts.tv_sec) * 1e6 + static_cast<double>(ts.tv_nsec) * 1e-3;
}

static inline double ts_diff_us(const struct timespec& a, const struct timespec& b) {
    return (static_cast<double>(a.tv_sec)  - static_cast<double>(b.tv_sec))  * 1e6
         + (static_cast<double>(a.tv_nsec) - static_cast<double>(b.tv_nsec)) * 1e-3;
}

// ---------------------------------------------------------------------------
V4L2Capture::V4L2Capture(const CameraConfig& config)
    : config_(config)
{
    pthread_mutex_init(&frame_mutex_, nullptr);
    pthread_cond_init(&frame_cond_,  nullptr);
    pthread_mutex_init(&stats_mutex_, nullptr);
}

V4L2Capture::~V4L2Capture() {
    if (running_) stop();
    pthread_mutex_destroy(&frame_mutex_);
    pthread_cond_destroy(&frame_cond_);
    pthread_mutex_destroy(&stats_mutex_);
}

void V4L2Capture::setFrameCallback(FrameCallback cb) {
    frame_cb_ = std::move(cb);
}

// ---------------------------------------------------------------------------
void V4L2Capture::start() {
    openDevice();
    setControls();
    setFormat();
    initMmap();
    startStreaming();

    running_.store(true, std::memory_order_release);

    // Publish thread – normal priority, no affinity constraint.
    pthread_attr_t pub_attr;
    pthread_attr_init(&pub_attr);
    if (pthread_create(&publish_thread_, &pub_attr, publishThreadEntry, this) != 0)
        throw std::runtime_error("Failed to create publish thread");
    pthread_attr_destroy(&pub_attr);

    // Capture thread – SCHED_FIFO, pinned to config_.cpu_core.
    pthread_attr_t cap_attr;
    pthread_attr_init(&cap_attr);
    pthread_attr_setinheritsched(&cap_attr, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&cap_attr, SCHED_FIFO);
    struct sched_param sp{};
    sp.sched_priority = config_.rt_priority;
    pthread_attr_setschedparam(&cap_attr, &sp);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(config_.cpu_core, &cpuset);
    pthread_attr_setaffinity_np(&cap_attr, sizeof(cpuset), &cpuset);

    int rc = pthread_create(&capture_thread_, &cap_attr, captureThreadEntry, this);
    if (rc != 0) {
        ROS_WARN("Could not create SCHED_FIFO thread (%s). "
                 "Re-trying without RT priority – run as root or set RLIMIT_RTPRIO.",
                 strerror(rc));
        pthread_attr_setschedpolicy(&cap_attr, SCHED_OTHER);
        sp.sched_priority = 0;
        pthread_attr_setschedparam(&cap_attr, &sp);
        if (pthread_create(&capture_thread_, &cap_attr, captureThreadEntry, this) != 0) {
            pthread_attr_destroy(&cap_attr);
            throw std::runtime_error("Failed to create capture thread");
        }
    }
    pthread_attr_destroy(&cap_attr);
}

void V4L2Capture::stop() {
    running_.store(false, std::memory_order_release);

    // Wake publish thread from condvar wait.
    pthread_mutex_lock(&frame_mutex_);
    pthread_cond_broadcast(&frame_cond_);
    pthread_mutex_unlock(&frame_mutex_);

    pthread_join(capture_thread_, nullptr);
    pthread_join(publish_thread_, nullptr);

    stopStreaming();
    unmapBuffers();
    if (epoll_fd_ >= 0) { ::close(epoll_fd_); epoll_fd_ = -1; }
    closeDevice();
}

// ---------------------------------------------------------------------------
void V4L2Capture::openDevice() {
    fd_ = ::open(config_.device.c_str(), O_RDWR | O_NONBLOCK);
    if (fd_ < 0)
        throw std::runtime_error("Cannot open " + config_.device + ": " + strerror(errno));

    struct v4l2_capability cap{};
    if (xioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
        throw std::runtime_error("VIDIOC_QUERYCAP failed");
    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
        throw std::runtime_error("Not a video capture device");
    if (!(cap.capabilities & V4L2_CAP_STREAMING))
        throw std::runtime_error("Device does not support streaming I/O");

    ROS_INFO("Camera opened: %s  driver: %s  bus: %s",
             cap.card, cap.driver, cap.bus_info);

    // Setup epoll for non-blocking dequeue.
    epoll_fd_ = epoll_create1(EPOLL_CLOEXEC);
    if (epoll_fd_ < 0)
        throw std::runtime_error("epoll_create1 failed: " + std::string(strerror(errno)));
    struct epoll_event ev{};
    ev.events  = EPOLLIN | EPOLLET;
    ev.data.fd = fd_;
    if (epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd_, &ev) < 0)
        throw std::runtime_error("epoll_ctl failed: " + std::string(strerror(errno)));
}

// ---------------------------------------------------------------------------
void V4L2Capture::setControl(uint32_t cid, int32_t value, const char* name) {
    struct v4l2_control ctrl{};
    ctrl.id    = cid;
    ctrl.value = value;
    if (xioctl(fd_, VIDIOC_S_CTRL, &ctrl) < 0)
        ROS_WARN("setControl %s (0x%08x = %d) failed: %s", name, cid, value, strerror(errno));
    else
        ROS_DEBUG("setControl %s = %d  OK", name, value);
}

void V4L2Capture::setControls() {
    // Disable auto features FIRST; driver may ignore absolute values if auto is on.
    setControl(V4L2_CID_EXPOSURE_AUTO,      1, "exposure_auto (1=manual)");
    setControl(V4L2_CID_FOCUS_AUTO,         0, "focus_auto");
    setControl(V4L2_CID_AUTO_WHITE_BALANCE, 0, "auto_white_balance");

    // Lock concrete values.
    setControl(V4L2_CID_EXPOSURE_ABSOLUTE,         config_.exposure_absolute,        "exposure_absolute");
    setControl(V4L2_CID_GAIN,                      config_.gain,                     "gain");
    setControl(V4L2_CID_WHITE_BALANCE_TEMPERATURE, config_.white_balance_temperature,"white_balance_temperature");
    setControl(V4L2_CID_FOCUS_ABSOLUTE,            config_.focus_absolute,           "focus_absolute");
    setControl(V4L2_CID_BRIGHTNESS,                config_.brightness,               "brightness");
    setControl(V4L2_CID_CONTRAST,                  config_.contrast,                 "contrast");
    setControl(V4L2_CID_SATURATION,                config_.saturation,               "saturation");
    setControl(V4L2_CID_SHARPNESS,                 config_.sharpness,                "sharpness");
    setControl(V4L2_CID_BACKLIGHT_COMPENSATION,    config_.backlight_compensation,   "backlight_compensation");
    setControl(V4L2_CID_POWER_LINE_FREQUENCY,      config_.power_line_frequency,     "power_line_frequency");

    ROS_INFO("All camera auto-controls disabled; fixed values applied.");
}

// ---------------------------------------------------------------------------
void V4L2Capture::setFormat() {
    struct v4l2_format fmt{};
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = config_.width;
    fmt.fmt.pix.height      = config_.height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

    if (xioctl(fd_, VIDIOC_S_FMT, &fmt) < 0)
        throw std::runtime_error("VIDIOC_S_FMT failed: " + std::string(strerror(errno)));

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_YUYV)
        throw std::runtime_error("Driver did not accept YUYV; got " +
                                 std::to_string(fmt.fmt.pix.pixelformat));

    if (fmt.fmt.pix.width != config_.width || fmt.fmt.pix.height != config_.height) {
        ROS_WARN("Driver adjusted resolution: requested %ux%u, got %ux%u",
                 config_.width, config_.height, fmt.fmt.pix.width, fmt.fmt.pix.height);
        config_.width  = fmt.fmt.pix.width;
        config_.height = fmt.fmt.pix.height;
    }
    ROS_INFO("Format: %ux%u YUYV  stride=%u  imagesize=%u",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             fmt.fmt.pix.bytesperline, fmt.fmt.pix.sizeimage);

    // Request 30 fps.
    struct v4l2_streamparm parm{};
    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_G_PARM, &parm) == 0) {
        parm.parm.capture.timeperframe.numerator   = 1;
        parm.parm.capture.timeperframe.denominator = 30;
        if (xioctl(fd_, VIDIOC_S_PARM, &parm) < 0)
            ROS_WARN("VIDIOC_S_PARM (fps) failed: %s", strerror(errno));
        else
            ROS_INFO("Frame rate: %u/%u fps",
                     parm.parm.capture.timeperframe.denominator,
                     parm.parm.capture.timeperframe.numerator);
    }
}

// ---------------------------------------------------------------------------
void V4L2Capture::initMmap() {
    struct v4l2_requestbuffers reqbuf{};
    reqbuf.count  = config_.num_buffers;
    reqbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    reqbuf.memory = V4L2_MEMORY_MMAP;

    if (xioctl(fd_, VIDIOC_REQBUFS, &reqbuf) < 0)
        throw std::runtime_error("VIDIOC_REQBUFS failed: " + std::string(strerror(errno)));
    if (reqbuf.count < 2)
        throw std::runtime_error("Insufficient buffer memory in driver");

    ROS_INFO("Driver allocated %u mmap buffers", reqbuf.count);
    mmap_bufs_.resize(reqbuf.count);

    for (uint32_t i = 0; i < reqbuf.count; ++i) {
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(fd_, VIDIOC_QUERYBUF, &buf) < 0)
            throw std::runtime_error("VIDIOC_QUERYBUF[" + std::to_string(i) + "] failed");

        mmap_bufs_[i].length = buf.length;
        mmap_bufs_[i].start  = static_cast<uint8_t*>(
            ::mmap(nullptr, buf.length,
                   PROT_READ | PROT_WRITE, MAP_SHARED,
                   fd_, buf.m.offset));
        if (mmap_bufs_[i].start == MAP_FAILED)
            throw std::runtime_error("mmap failed for buffer " + std::to_string(i) +
                                     ": " + strerror(errno));

        // Pre-fault pages to avoid page faults during realtime capture.
        volatile uint8_t* p = mmap_bufs_[i].start;
        for (size_t off = 0; off < buf.length; off += 4096)
            (void)p[off];
    }

    // Queue all buffers into the driver.
    for (uint32_t i = 0; i < reqbuf.count; ++i) {
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0)
            throw std::runtime_error("VIDIOC_QBUF[" + std::to_string(i) + "] failed");
    }
}

void V4L2Capture::startStreaming() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd_, VIDIOC_STREAMON, &type) < 0)
        throw std::runtime_error("VIDIOC_STREAMON failed: " + std::string(strerror(errno)));
    ROS_INFO("Streaming started.");
}

void V4L2Capture::stopStreaming() {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    xioctl(fd_, VIDIOC_STREAMOFF, &type);
}

void V4L2Capture::unmapBuffers() {
    for (auto& b : mmap_bufs_)
        if (b.start && b.start != MAP_FAILED)
            ::munmap(b.start, b.length);
    mmap_bufs_.clear();
}

void V4L2Capture::closeDevice() {
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

// ---------------------------------------------------------------------------
// Capture thread – SCHED_FIFO, pinned CPU.
// Critical path per frame:
//   epoll_wait  →  VIDIOC_DQBUF  →  clock_gettime(CLOCK_MONOTONIC_RAW)
//   →  hand to publish thread (mutex lock/unlock + condvar signal)
// ---------------------------------------------------------------------------
void* V4L2Capture::captureThreadEntry(void* arg) {
    static_cast<V4L2Capture*>(arg)->captureLoop();
    return nullptr;
}

void V4L2Capture::captureLoop() {
    // Lock all current and future memory to prevent page faults.
    if (mlockall(MCL_CURRENT | MCL_FUTURE) < 0)
        ROS_WARN("mlockall failed: %s  (run as root for best RT behaviour)", strerror(errno));

    {
        // Log actual scheduling policy achieved.
        int policy;
        struct sched_param sp{};
        pthread_getschedparam(pthread_self(), &policy, &sp);
        ROS_INFO("Capture thread: policy=%s priority=%d cpu=%d",
                 (policy == SCHED_FIFO) ? "SCHED_FIFO" : "SCHED_OTHER",
                 sp.sched_priority, config_.cpu_core);
    }

    struct epoll_event ev;
    bool logged_ts_type = false;

    while (running_.load(std::memory_order_relaxed)) {
        int n = epoll_wait(epoll_fd_, &ev, 1, 200 /*ms timeout*/);
        if (!running_.load(std::memory_order_relaxed)) break;

        if (n < 0) {
            if (errno == EINTR) continue;
            ROS_ERROR("epoll_wait: %s", strerror(errno));
            break;
        }
        if (n == 0) continue; // timeout – no frame yet

        // --- Dequeue frame (non-blocking; epoll confirmed data ready) ---
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        if (xioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) continue;
            ROS_ERROR("VIDIOC_DQBUF: %s", strerror(errno));
            break;
        }

        // --- Capture timestamp – CLOCK_MONOTONIC_RAW immediately on dequeue ---
        struct timespec ts_raw{};
        clock_gettime(CLOCK_MONOTONIC_RAW, &ts_raw);

        bool driver_mono = (buf.flags & V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC) != 0;
        if (!logged_ts_type) {
            ROS_INFO("Driver timestamp type: %s",
                     driver_mono ? "CLOCK_MONOTONIC" : "CLOCK_REALTIME/unknown");
            logged_ts_type = true;
        }

        // --- Update inter-frame jitter stats (Welford online algorithm) ---
        pthread_mutex_lock(&stats_mutex_);
        if (!first_frame_) {
            double interval_us = ts_diff_us(ts_raw, last_frame_ts_);
            stats_.frames_captured++;
            double delta     = interval_us - mean_interval_us_;
            mean_interval_us_ += delta / static_cast<double>(stats_.frames_captured);
            M2_interval_      += delta * (interval_us - mean_interval_us_);
            stats_.jitter_us  = (stats_.frames_captured > 1)
                                ? sqrt(M2_interval_ / static_cast<double>(stats_.frames_captured))
                                : 0.0;
            stats_.mean_interval_us = mean_interval_us_;
            if (interval_us > stats_.max_interval_us) stats_.max_interval_us = interval_us;
            if (interval_us < stats_.min_interval_us) stats_.min_interval_us = interval_us;
        } else {
            first_frame_ = false;
            stats_.frames_captured = 1;
        }
        last_frame_ts_ = ts_raw;
        pthread_mutex_unlock(&stats_mutex_);

        // --- Hand frame to publish thread; drop previous if still pending ---
        pthread_mutex_lock(&frame_mutex_);
        if (shared_frame_.valid) {
            // Publish thread hasn't consumed the last frame – drop it and
            // immediately requeue that buffer so the driver doesn't starve.
            struct v4l2_buffer old{};
            old.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            old.memory = V4L2_MEMORY_MMAP;
            old.index  = shared_frame_.buf_index;
            xioctl(fd_, VIDIOC_QBUF, &old);

            pthread_mutex_lock(&stats_mutex_);
            stats_.frames_dropped++;
            pthread_mutex_unlock(&stats_mutex_);
        }
        shared_frame_.valid           = true;
        shared_frame_.buf_index       = buf.index;
        shared_frame_.bytesused       = buf.bytesused;
        shared_frame_.timestamp       = ts_raw;
        shared_frame_.sequence        = buf.sequence;
        shared_frame_.driver_monotonic = driver_mono;
        shared_frame_.driver_ts       = buf.timestamp;
        pthread_cond_signal(&frame_cond_);
        pthread_mutex_unlock(&frame_mutex_);
    }

    ROS_INFO("Capture thread exiting.");
}

// ---------------------------------------------------------------------------
// Publish thread – normal priority.
// Reads directly from the mmap buffer (zero copy) before re-queuing.
// ---------------------------------------------------------------------------
void* V4L2Capture::publishThreadEntry(void* arg) {
    static_cast<V4L2Capture*>(arg)->publishLoop();
    return nullptr;
}

void V4L2Capture::publishLoop() {
    ROS_INFO("Publish thread started.");

    while (true) {
        SharedFrame frame;

        pthread_mutex_lock(&frame_mutex_);
        while (!shared_frame_.valid && running_.load(std::memory_order_relaxed))
            pthread_cond_wait(&frame_cond_, &frame_mutex_);

        if (!running_.load(std::memory_order_relaxed) && !shared_frame_.valid) {
            pthread_mutex_unlock(&frame_mutex_);
            break;
        }
        frame = shared_frame_;
        shared_frame_.valid = false;
        pthread_mutex_unlock(&frame_mutex_);

        if (frame_cb_) {
            FrameData fd_out{};
            fd_out.data             = mmap_bufs_[frame.buf_index].start;
            fd_out.bytesused        = frame.bytesused;
            fd_out.width            = config_.width;
            fd_out.height           = config_.height;
            fd_out.timestamp        = frame.timestamp;
            fd_out.sequence         = frame.sequence;
            fd_out.driver_monotonic = frame.driver_monotonic;
            fd_out.driver_ts        = frame.driver_ts;
            frame_cb_(fd_out);
        }

        // Return buffer to driver so it can receive the next frame.
        struct v4l2_buffer buf{};
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = frame.buf_index;
        if (xioctl(fd_, VIDIOC_QBUF, &buf) < 0)
            ROS_ERROR("VIDIOC_QBUF (publish thread): %s", strerror(errno));
    }

    ROS_INFO("Publish thread exiting.");
}

// ---------------------------------------------------------------------------
CaptureStats V4L2Capture::getStats() const {
    pthread_mutex_lock(&stats_mutex_);
    CaptureStats s = stats_;
    pthread_mutex_unlock(&stats_mutex_);
    return s;
}
