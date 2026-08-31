#include "../include/sensors.hpp"
#include <iostream>
#include <stdexcept>
#include <thread>
#include <chrono>

// X11 window capture is a desktop-only facility.  USE_X11 is defined by CMake when
// libX11 is found; a headless host (the Pi) builds ogma_infra without it and
// create_window_video() throws instead.
#ifdef USE_X11
#include <X11/Xlib.h>
#include <X11/Xutil.h>

namespace ami_ogma {
namespace sensors {

class X11WindowCaptureStream : public VideoStream {
public:
    X11WindowCaptureStream(const std::string& window_id_str, size_t width, size_t height) 
        : width_(width), height_(height) {
        
        display_ = XOpenDisplay(NULL);
        if (!display_) {
            throw std::runtime_error("Failed to open X11 Display.");
        }
        
        // Handle window_id as hex if starts with 0x, otherwise base 10
        int base = 10;
        if (window_id_str.find("0x") == 0 || window_id_str.find("0X") == 0) {
            base = 16;
        }
        
        try {
            window_ = std::stoul(window_id_str, nullptr, base);
        } catch (const std::exception& e) {
            XCloseDisplay(display_);
            throw std::runtime_error("Invalid window ID provided: " + window_id_str);
        }

        std::cout << "Initialized X11WindowCaptureStream (Window " << std::hex << window_ << std::dec << ")" << std::endl;
        
        fps_ = 30.0;
        frame_interval_ms_ = 1000.0 / fps_;
        last_read_time_ = std::chrono::steady_clock::now();
    }
    
    ~X11WindowCaptureStream() {
        if (display_) {
            XCloseDisplay(display_);
        }
    }

    bool read_frame(std::vector<float>& out_tensor) override {
        // Enforce frame rate
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_time_).count();
        if (elapsed_ms < frame_interval_ms_) {
            std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(frame_interval_ms_ - elapsed_ms)));
        }
        last_read_time_ = std::chrono::steady_clock::now();

        if (!display_) return false;

        XWindowAttributes attr;
        if (!XGetWindowAttributes(display_, window_, &attr)) {
            std::cerr << "Failed to get window attributes for: " << std::hex << window_ << std::dec << std::endl;
            return false;
        }

        // Use coordinate translation to get relative-to-root positioning
        // This is necessary because in composited environments, window-local XGetImage often returns black
        int root_x, root_y;
        Window child;
        XTranslateCoordinates(display_, window_, attr.root, 0, 0, &root_x, &root_y, &child);

        // Capture from the root window at the absolute coordinates
        XImage* img = XGetImage(display_, attr.root, root_x, root_y, 
                                attr.width, attr.height, AllPlanes, ZPixmap);
                                
        if (!img) {
            // Fallback: Try capturing from the window directly if root capture fails (e.g. if root is protected)
            img = XGetImage(display_, window_, 0, 0, 
                            attr.width, attr.height, AllPlanes, ZPixmap);
        }

        if (!img) {
            std::cerr << "Failed to grab X11 image from Root or Window." << std::endl;
            return false;
        }

        size_t expected_size = width_ * height_ * 3;
        if (out_tensor.size() != expected_size) {
            out_tensor.resize(expected_size);
        }

        float* dst = out_tensor.data();
        size_t plane_size = height_ * width_;
        
        uint8_t* raw_data = reinterpret_cast<uint8_t*>(img->data);
        int src_w = attr.width;
        int src_h = attr.height;
        int bytes_per_line = img->bytes_per_line;
        int bpp = img->bits_per_pixel / 8;
        
        // Pure C++ Nearest-Neighbor Resize + BGRA to Planar RGB Conversion
        for (size_t y = 0; y < height_; ++y) {
            int src_y = (y * src_h) / height_;
            for (size_t x = 0; x < width_; ++x) {
                int src_x = (x * src_w) / width_;
                
                int src_idx = src_y * bytes_per_line + src_x * bpp;
                uint8_t b = raw_data[src_idx];
                uint8_t g = raw_data[src_idx + 1];
                uint8_t r = raw_data[src_idx + 2];
                // Ignored alpha raw_data[src_idx + 3];
                
                int dst_idx = y * width_ + x;
                dst[dst_idx]                  = r / 255.0f; // R plane
                dst[plane_size + dst_idx]     = g / 255.0f; // G plane
                dst[2 * plane_size + dst_idx] = b / 255.0f; // B plane
            }
        }

        XDestroyImage(img);
        return true;
    }

private:
    Display* display_;
    Window window_;
    size_t width_;
    size_t height_;
    double fps_;
    double frame_interval_ms_;
    std::chrono::steady_clock::time_point last_read_time_;
};

} // namespace sensors
} // namespace ami_ogma
#endif // USE_X11

// Conditional compilation for OpenCV to allow audio-only builds if OpenCV is missing
#ifdef USE_OPENCV
// X11 #defines Status/Bool/None as macros that clash with OpenCV enums — undef first.
#ifdef Status
#undef Status
#endif
#ifdef Bool
#undef Bool
#endif
#ifdef None
#undef None
#endif
#include <opencv2/opencv.hpp>

namespace ami_ogma {
namespace sensors {


class OpenCVVideoStream : public VideoStream {
public:
    OpenCVVideoStream(const std::string& source, size_t width, size_t height, bool is_mock) 
        : width_(width), height_(height), is_mock_(is_mock) {
        
        // Treat source as an integer (device ID) for live, or a string file path for mock
        if (!is_mock) {
            int device_id = std::stoi(source);
            cap_.open(device_id, cv::CAP_V4L2); // Force V4L2 for robust Linux webcam support
            std::cout << "Initialized LiveVideoStream (Device " << device_id << ")" << std::endl;
        } else {
            cap_.open(source);
            std::cout << "Initialized MockVideoStream (File: " << source << ")" << std::endl;
        }

        if (!cap_.isOpened()) {
            throw std::runtime_error("Failed to open OpenCV video source: " + source);
        }
        
        // Attempt to set camera resolution if live
        if (!is_mock) {
            cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
            cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        }
        
        fps_ = cap_.get(cv::CAP_PROP_FPS);
        if (fps_ <= 0) fps_ = 30.0; // Default fallback
        frame_interval_ms_ = 1000.0 / fps_;
        last_read_time_ = std::chrono::steady_clock::now();
    }
    
    ~OpenCVVideoStream() {
        if (cap_.isOpened()) cap_.release();
    }

    bool read_frame(std::vector<float>& out_tensor) override {
        // Enforce Real-Time Simulation Delay for mock files so they don't process at maximum CPU speed
        if (is_mock_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_read_time_).count();
            if (elapsed_ms < frame_interval_ms_) {
                std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(frame_interval_ms_ - elapsed_ms)));
            }
            last_read_time_ = std::chrono::steady_clock::now();
        }

        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            std::cout << "Video Stream ended or frame empty." << std::endl;
            return false;
        }

        // Processing Pipeline:
        // 1. Resize to expected ONNX input dimensions
        cv::Mat resized;
        cv::resize(frame, resized, cv::Size(width_, height_));
        
        // 2. Convert to RGB (OpenCV uses BGR natively)
        cv::Mat rgb;
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB);

        size_t expected_size = width_ * height_ * 3;
        if (out_tensor.size() != expected_size) {
            out_tensor.resize(expected_size);
        }

        // Convert [H, W, C] to Planar [C, H, W]
        float* dst = out_tensor.data();
        size_t plane_size = height_ * width_;
        
        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                cv::Vec3b pixel = rgb.at<cv::Vec3b>(y, x);
                int idx = y * width_ + x;
                dst[idx]                  = pixel[0] / 255.0f; // R plane
                dst[plane_size + idx]     = pixel[1] / 255.0f; // G plane
                dst[2 * plane_size + idx] = pixel[2] / 255.0f; // B plane
            }
        }

        return true;
    }

private:
    cv::VideoCapture cap_;
    size_t width_;
    size_t height_;
    bool is_mock_;
    double fps_;
    double frame_interval_ms_;
    std::chrono::steady_clock::time_point last_read_time_;
};



// ==========================================
// Factories (OpenCV Enabled)
// ==========================================

std::unique_ptr<VideoStream> create_live_video(int device_id, size_t width, size_t height) {
    return std::make_unique<OpenCVVideoStream>(std::to_string(device_id), width, height, false);
}

std::unique_ptr<VideoStream> create_mock_video(const std::string& mp4_file_path, size_t width, size_t height) {
    return std::make_unique<OpenCVVideoStream>(mp4_file_path, width, height, true);
}

std::unique_ptr<VideoStream> create_window_video(const std::string& window_id, size_t width, size_t height) {
#ifdef USE_X11
    return std::make_unique<X11WindowCaptureStream>(window_id, width, height);
#else
    throw std::runtime_error("X11 not compiled into ogma_core. Window capture unimplemented.");
#endif
}

} // namespace sensors
} // namespace ami_ogma

#else // OpenCV Missing Dummy Implementations

namespace ami_ogma {
namespace sensors {

std::unique_ptr<VideoStream> create_live_video(int device_id, size_t width, size_t height) {
    throw std::runtime_error("OpenCV not compiled into ogma_core. Live Video unimplemented.");
}

std::unique_ptr<VideoStream> create_mock_video(const std::string& mp4_file_path, size_t width, size_t height) {
    throw std::runtime_error("OpenCV not compiled into ogma_core. Mock Video unimplemented.");
}

std::unique_ptr<VideoStream> create_window_video(const std::string& window_id, size_t width, size_t height) {
#ifdef USE_X11
    // We can use our pure X11 fallback!
    return std::make_unique<X11WindowCaptureStream>(window_id, width, height);
#else
    throw std::runtime_error("X11 not compiled into ogma_core. Window capture unimplemented.");
#endif
}

} // namespace sensors
} // namespace ami_ogma

#endif
