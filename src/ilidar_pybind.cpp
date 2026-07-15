/**
 * @file ilidar_pybind.cpp
 * @brief Python bindings for the iTFS LiDAR C++ SDK
 * @author Junwoo Son (json@hybo.co)
 * @date 2026-07-09
 * @version 2.0.0
 */

//////////////////////////////////////////////////////////////////////////////////////
//    MIT License (MIT)                                                             //
//                                                                                  //
//    Copyright (c) 2022 - Present HYBO Inc.                                        //
//                                                                                  //
//    Permission is hereby granted, free of charge, to any person obtaining a copy  //
//    of this software and associated documentation files (the "Software"),         //
//    to deal in the Software without restriction, including without limitation     //
//    the rights to use, copy, modify, merge, publish, distribute, sublicense,      //
//    and/or sell copies of the Software, and to permit persons to whom the         //
//    Software is furnished to do so, subject to the following conditions:          //
//                                                                                  //
//    The above copyright notice and this permission notice shall be included in    //
//    all copies or substantial portions of the Software.                           //
//                                                                                  //
//    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR    //
//    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,      //
//    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL       //
//    THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER    //
//    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, //
//    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN     //
//    THE SOFTWARE.                                                                 //
//////////////////////////////////////////////////////////////////////////////////////

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "ilidar.hpp"

namespace py = pybind11;

namespace {

constexpr int kRows = iTFS::max_row;
constexpr int kCols = iTFS::max_col;

// Native event and image snapshots stored independently of SDK callback memory.
struct Frame {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    uint8_t mode = 0;
    uint8_t frame_counter = 0;
    int frame_status = 0;
    int capture_row = kRows;
    bool depth_on = true;
    bool intensity_on = false;
    iTFS::packet::status_t status{};
    iTFS::packet::status_full_t status_full{};
    iTFS::packet::info_t info{};
    iTFS::packet::info_v2_t info_v2{};
    std::vector<uint16_t> image;
};

struct StatusEvent {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    iTFS::packet::status_t status{};
    iTFS::packet::status_full_t status_full{};
};

struct InfoEvent {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    int version = 0;
    iTFS::packet::info_t info{};
    iTFS::packet::info_v2_t info_v2{};
};

struct DeviceSnapshot {
    int index = -1;
    std::string ip;
    uint16_t port = 0;
    iTFS::packet::status_t status{};
    iTFS::packet::status_full_t status_full{};
    iTFS::packet::info_t info{};
    iTFS::packet::info_v2_t info_v2{};
};

struct Img {
    iTFS::img_t image{};
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    uint8_t data_output = 0;
};

class LiDAR;

std::mutex g_active_mutex;
LiDAR *g_active_lidar = nullptr;

// Common network helpers used while opening the native receive runtime.
std::string ip_to_string(const uint8_t ip[4]) {
    return std::to_string(ip[0]) + "." + std::to_string(ip[1]) + "." +
           std::to_string(ip[2]) + "." + std::to_string(ip[3]);
}

std::array<uint8_t, 4> parse_ip(const std::string &ip) {
    std::array<uint8_t, 4> out{};
    size_t start = 0;
    for (int i = 0; i < 4; i++) {
        size_t end = ip.find('.', start);
        std::string part = ip.substr(start, end == std::string::npos ? end : end - start);
        if (part.empty()) {
            throw std::invalid_argument("invalid IPv4 address: " + ip);
        }
        int value = std::stoi(part);
        if (value < 0 || value > 255) {
            throw std::invalid_argument("invalid IPv4 address: " + ip);
        }
        out[i] = static_cast<uint8_t>(value);
        if (i < 3 && end == std::string::npos) {
            throw std::invalid_argument("invalid IPv4 address: " + ip);
        }
        start = end + 1;
    }
    if (start < ip.size() + 1 && ip.find('.', start) != std::string::npos) {
        throw std::invalid_argument("invalid IPv4 address: " + ip);
    }
    return out;
}

void preflight_receiver_socket(const uint8_t *listening_ip, uint16_t listening_port) {
#if !defined(_WIN32) && !defined(_WIN64)
    SOCKET_TYPE sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (SOCKET_INVALID(sockfd)) {
        throw std::runtime_error("failed to open UDP receiver socket");
    }

    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<char *>(&enable), sizeof(enable)) == SOCKET_ERROR_RET) {
        closesocket(sockfd);
        throw std::runtime_error("failed to configure UDP receiver socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(listening_port);
    if (listening_ip == nullptr) {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        char addr_str[32];
        std::snprintf(addr_str, sizeof(addr_str), "%u.%u.%u.%u",
                      listening_ip[0], listening_ip[1], listening_ip[2], listening_ip[3]);
        addr.sin_addr.s_addr = inet_addr(addr_str);
    }

    if (bind(sockfd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == SOCKET_ERROR_RET) {
        closesocket(sockfd);
        throw std::runtime_error("failed to bind UDP receiver socket");
    }

    closesocket(sockfd);
#else
    (void)listening_ip;
    (void)listening_port;
#endif
}

std::string fixed_char_string(const char *data, size_t size) {
    size_t len = 0;
    while (len < size && data[len] != '\0') {
        len++;
    }
    return std::string(data, len);
}

// iTFS image layout and conversion helpers.
py::object image_to_numpy(const Frame &frame, int row_offset, int rows) {
    if (frame.image.empty() || rows <= 0) {
        return py::none();
    }
    py::array_t<uint16_t> arr({rows, kCols});
    const uint16_t *src = frame.image.data() + static_cast<size_t>(row_offset) * kCols;
    std::memcpy(arr.mutable_data(), src, static_cast<size_t>(rows) * kCols * sizeof(uint16_t));
    return arr;
}

py::object depth_to_numpy(const Frame &frame) {
    int rows = frame.capture_row;
    if (rows <= 0 || rows > kRows) {
        rows = kRows;
    }
    return image_to_numpy(frame, 0, rows);
}

py::object intensity_to_numpy(const Frame &frame) {
    if (!frame.intensity_on || frame.mode == 0) {
        return py::none();
    }
    int rows = frame.capture_row;
    if (rows > kRows) {
        rows = kRows;
    }
    return image_to_numpy(frame, kRows, rows);
}

py::dict status_to_dict(const iTFS::packet::status_t &s) {
    py::dict d;
    d["capture_mode"] = s.capture_mode;
    d["capture_frame"] = s.capture_frame;
    d["sensor_sn"] = s.sensor_sn;
    d["sensor_time_us"] = iTFS::packet::get_sensor_time_in_us(s.sensor_time_th, s.sensor_time_tl);
    d["sensor_frame_status"] = s.sensor_frame_status;
    d["temp_rx_c"] = s.sensor_temp_rx * 0.01;
    d["temp_core_c"] = s.sensor_temp_core * 0.01;
    d["vcsel_level_v"] = s.sensor_vcsel_level * 0.01;
    d["power_level_v"] = s.sensor_power_level * 0.01;
    d["warning"] = s.sensor_warning;
    return d;
}

py::dict status_full_to_dict(const iTFS::packet::status_full_t &s) {
    py::dict d;
    d["capture_mode"] = s.capture_mode;
    d["capture_frame"] = s.capture_frame;
    d["sensor_sn"] = s.sensor_sn;
    d["sensor_time_us"] = iTFS::packet::get_sensor_time_in_us(s.sensor_time_th, s.sensor_time_tl);
    d["sensor_frame_status"] = s.sensor_frame_status;
    d["temp_rx_c"] = s.sensor_temp_rx * 0.01;
    d["temp_core_c"] = s.sensor_temp_core * 0.01;
    d["temp_c"] = py::make_tuple(s.sensor_temp[0] * 0.01, s.sensor_temp[1] * 0.01, s.sensor_temp[2] * 0.01, s.sensor_temp[3] * 0.01);
    d["vcsel_level_v"] = s.sensor_vcsel_level * 0.01;
    d["power_level_v"] = s.sensor_power_level * 0.01;
    py::list vcsel_on;
    py::list power_on;
    for (int r = 0; r < 4; r++) {
        py::list vcsel_row;
        py::list power_row;
        for (int c = 0; c < 16; c++) {
            vcsel_row.append(s.sensor_vcsel_on[r][c] * 0.01);
            power_row.append(s.sensor_power_on[r][c] * 0.01);
        }
        vcsel_on.append(vcsel_row);
        power_on.append(power_row);
    }
    py::list levels;
    for (int i = 0; i < 10; i++) {
        levels.append(s.sensor_level[i] * 0.01);
    }
    d["vcsel_on_v"] = vcsel_on;
    d["power_on_v"] = power_on;
    d["level_v"] = levels;
    d["warning"] = s.sensor_warning;
    return d;
}

py::dict status_event_to_dict(const StatusEvent &event) {
    py::dict d = status_to_dict(event.status);
    d["idx"] = event.device_index;
    d["ip"] = event.ip;
    d["port"] = event.port;
    d["status_full"] = status_full_to_dict(event.status_full);
    d["status_full_available"] = event.status_full.sensor_sn != 0;
    return d;
}

py::dict info_to_dict(const iTFS::packet::info_t &info) {
    py::dict d;
    d["version"] = 1;
    d["sensor_sn"] = info.sensor_sn;
    d["sensor_hw_id"] = fixed_char_string(reinterpret_cast<const char *>(info.sensor_hw_id), 30);
    d["sensor_fw_ver"] = py::make_tuple(info.sensor_fw_ver[2], info.sensor_fw_ver[1], info.sensor_fw_ver[0]);
    d["sensor_fw_date"] = fixed_char_string(info.sensor_fw_date, 12);
    d["sensor_fw_time"] = fixed_char_string(info.sensor_fw_time, 9);
    d["sensor_calib_id"] = info.sensor_calib_id;
    d["capture_mode"] = info.capture_mode;
    d["capture_row"] = info.capture_row;
    d["capture_period"] = info.capture_period;
    d["capture_shutter"] = py::make_tuple(info.capture_shutter[0], info.capture_shutter[1], info.capture_shutter[2], info.capture_shutter[3], info.capture_shutter[4]);
    d["capture_limit"] = py::make_tuple(info.capture_limit[0], info.capture_limit[1]);
    d["data_output"] = info.data_output;
    d["data_sensor_ip"] = ip_to_string(info.data_sensor_ip);
    d["data_dest_ip"] = ip_to_string(info.data_dest_ip);
    d["data_port"] = info.data_port;
    d["sync"] = info.sync;
    d["sync_delay"] = info.sync_delay;
    d["arb"] = info.arb;
    d["arb_timeout"] = info.arb_timeout;
    d["lock"] = info.lock;
    return d;
}

py::dict info_v2_to_dict(const iTFS::packet::info_v2_t &info) {
    py::dict d;
    d["version"] = 2;
    d["sensor_sn"] = info.sensor_sn;
    d["sensor_hw_id"] = fixed_char_string(reinterpret_cast<const char *>(info.sensor_hw_id), 30);
    d["sensor_fw_ver"] = py::make_tuple(info.sensor_fw_ver[2], info.sensor_fw_ver[1], info.sensor_fw_ver[0]);
    d["sensor_fw_date"] = fixed_char_string(info.sensor_fw_date, 12);
    d["sensor_fw_time"] = fixed_char_string(info.sensor_fw_time, 9);
    d["sensor_calib_id"] = info.sensor_calib_id;
    d["sensor_fw0_ver"] = py::make_tuple(info.sensor_fw0_ver[2], info.sensor_fw0_ver[1], info.sensor_fw0_ver[0]);
    d["sensor_fw1_ver"] = py::make_tuple(info.sensor_fw1_ver[2], info.sensor_fw1_ver[1], info.sensor_fw1_ver[0]);
    d["sensor_fw2_ver"] = py::make_tuple(info.sensor_fw2_ver[2], info.sensor_fw2_ver[1], info.sensor_fw2_ver[0]);
    d["sensor_model_id"] = info.sensor_model_id;
    d["sensor_boot_mode"] = info.sensor_boot_mode;
    d["capture_mode"] = info.capture_mode;
    d["capture_row"] = info.capture_row;
    d["capture_period_us"] = info.capture_period_us;
    d["capture_seq"] = info.capture_seq;
    d["capture_shutter"] = py::make_tuple(info.capture_shutter[0], info.capture_shutter[1], info.capture_shutter[2], info.capture_shutter[3], info.capture_shutter[4]);
    d["capture_limit"] = py::make_tuple(info.capture_limit[0], info.capture_limit[1]);
    d["data_output"] = info.data_output;
    d["data_sensor_ip"] = ip_to_string(info.data_sensor_ip);
    d["data_dest_ip"] = ip_to_string(info.data_dest_ip);
    d["data_port"] = info.data_port;
    d["sync"] = info.sync;
    d["sync_trig_delay_us"] = info.sync_trig_delay_us;
    d["arb"] = info.arb;
    d["arb_timeout"] = info.arb_timeout;
    d["lock"] = info.lock;
    return d;
}

py::dict info_event_to_dict(const InfoEvent &event) {
    py::dict d;
    if (event.version == 1) {
        d = info_to_dict(event.info);
    } else if (event.version == 2) {
        d = info_v2_to_dict(event.info_v2);
    } else {
        d["version"] = 0;
        d["valid"] = false;
    }
    d["idx"] = event.device_index;
    d["ip"] = event.ip;
    d["port"] = event.port;
    return d;
}

int copied_rows(const Img &copy) {
    if (copy.image.capture_row <= 0 || copy.image.capture_row > kRows) {
        return kRows;
    }
    return copy.image.capture_row;
}

py::object copied_image_to_numpy(Img &copy,
                                 int row_offset,
                                 int rows,
                                 py::handle owner) {
    if (rows <= 0) {
        return py::none();
    }
    return py::array(
        py::dtype::of<uint16_t>(),
        {rows, kCols},
        {static_cast<py::ssize_t>(kCols * sizeof(uint16_t)),
         static_cast<py::ssize_t>(sizeof(uint16_t))},
        &copy.image.img[row_offset][0],
        owner);
}

py::object copied_depth_to_numpy(Img &copy, py::handle owner) {
    return copied_image_to_numpy(copy, 0, copied_rows(copy), owner);
}

py::object copied_intensity_to_numpy(Img &copy, py::handle owner) {
    bool intensity_on = (copy.data_output & iTFS::packet::data_output_intensity_mask) != 0;
    if (!intensity_on || copy.image.mode == 0) {
        return py::none();
    }
    return copied_image_to_numpy(copy, kRows, copied_rows(copy), owner);
}

py::array_t<uint8_t> copied_display_to_numpy(const Img &copy, int row_offset) {
    int rows = copied_rows(copy);
    py::array_t<uint8_t> output({rows, kCols});
    uint8_t *dst = output.mutable_data();
    const uint16_t *src = &copy.image.img[row_offset][0];
    size_t pixels = static_cast<size_t>(rows) * kCols;
    for (size_t i = 0; i < pixels; i++) {
        double scaled = src[i] * 255.0 / 3000.0;
        dst[i] = static_cast<uint8_t>(scaled >= 255.0 ? 255 : (scaled <= 0.0 ? 0 : scaled + 0.5));
    }
    return output;
}

py::array_t<float> copied_point_cloud_to_numpy(
    const Img &copy,
    py::array_t<float, py::array::c_style | py::array::forcecast> intrinsic) {
    if (intrinsic.ndim() != 3 || intrinsic.shape(0) != iTFS::gray_row ||
        intrinsic.shape(1) != kCols || intrinsic.shape(2) != 3) {
        throw std::invalid_argument("intrinsic must have shape (240, 320, 3)");
    }

    py::array_t<float> output({kRows * kCols, 3});
    const float *vectors = intrinsic.data() + ((iTFS::gray_row - kRows) / 2) * kCols * 3;
    const uint16_t *depth = &copy.image.img[0][0];
    float *points = output.mutable_data();

    for (int i = 0; i < kRows * kCols; i++) {
        float depth_m = depth[i] * 0.001f;
        points[3 * i + 0] = depth_m * vectors[3 * i + 2];
        points[3 * i + 1] = -depth_m * vectors[3 * i + 0];
        points[3 * i + 2] = -depth_m * vectors[3 * i + 1];
    }
    return output;
}

// Callback-only view of the SDK-owned device structure.
class Device {
  public:
    explicit Device(iTFS::device_t *device) : device_(device) {}

    void invalidate() { device_ = nullptr; }
    int device_index() const { return device()->idx; }
    std::string ip() const { return ip_to_string(device()->ip); }
    uint16_t port() const { return device()->port; }

    py::dict status() const {
        StatusEvent event;
        event.device_index = device()->idx;
        event.ip = ip_to_string(device()->ip);
        event.port = device()->port;
        event.status = device()->status;
        event.status_full = device()->status_full;
        return status_event_to_dict(event);
    }

    py::dict info() const {
        InfoEvent event;
        event.device_index = device()->idx;
        event.ip = ip_to_string(device()->ip);
        event.port = device()->port;
        event.info = device()->info;
        event.info_v2 = device()->info_v2;
        event.version = device()->info.sensor_sn != 0 ? 1 :
            (device()->info_v2.sensor_sn != 0 ? 2 : 0);
        return info_event_to_dict(event);
    }

    void copy_image(Img &copy) const {
        // Deep-copy the image buffer, matching the C++ OpenCV handler.
        copy.image.mode = device()->data.mode;
        copy.image.frame = device()->data.frame;
        copy.image.capture_row = device()->data.capture_row;
        copy.image.frame_status = device()->data.frame_status;
        std::memcpy(copy.image.img, device()->data.img, sizeof(device()->data.img));
        copy.device_index = device()->idx;
        copy.ip = ip_to_string(device()->ip);
        copy.port = device()->port;
        copy.data_output = device()->info.sensor_sn != 0 ?
            device()->info.data_output : device()->info_v2.data_output;
    }

  private:
    iTFS::device_t *device() const {
        if (device_ == nullptr) {
            throw std::runtime_error("Device is valid only while its event callback is running");
        }
        return device_;
    }

    iTFS::device_t *device_;
};

// Owns the native SDK runtime and exposes queued and direct callback APIs.
class LiDAR {
  public:
    LiDAR(size_t queue_size, py::object broadcast_ip, py::object listening_ip, uint16_t listening_port)
        : queue_size_(queue_size == 0 ? 1 : queue_size) {
        uint8_t *broadcast_ptr = nullptr;
        uint8_t *listen_ptr = nullptr;
        if (!broadcast_ip.is_none()) {
            broadcast_ip_ = parse_ip(py::cast<std::string>(broadcast_ip));
            broadcast_ptr = broadcast_ip_.data();
        }
        if (!listening_ip.is_none()) {
            listening_ip_ = parse_ip(py::cast<std::string>(listening_ip));
            listen_ptr = listening_ip_.data();
        }

        preflight_receiver_socket(listen_ptr, listening_port);

        {
            std::lock_guard<std::mutex> lk(g_active_mutex);
            if (g_active_lidar != nullptr) {
                throw std::runtime_error("only one ilidar.LiDAR instance can be active at a time");
            }
            g_active_lidar = this;
        }

        callback_thread_ = std::thread([this] { callback_loop(); });

        try {
            py::gil_scoped_release release;
            lidar_ = std::make_unique<iTFS::LiDAR>(
                &LiDAR::image_callback,
                &LiDAR::status_callback,
                &LiDAR::info_callback,
                broadcast_ptr,
                listen_ptr,
                listening_port);
        } catch (...) {
            close();
            throw;
        }
    }

    ~LiDAR() {
        close();
    }

    bool ready() const {
        return lidar_ && lidar_->Ready();
    }

    int device_count() const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        return static_cast<int>(devices_.size());
    }

    void close() {
        std::unique_ptr<iTFS::LiDAR> local_lidar;
        {
            std::lock_guard<std::mutex> lk(close_mutex_);
            if (closed_) {
                return;
            }
            closed_ = true;
            local_lidar = std::move(lidar_);
        }

        if (local_lidar) {
            py::gil_scoped_release release;
            local_lidar->Try_exit();
            local_lidar->Join();
            local_lidar.reset();
        }

        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            callback_ = py::object();
            callback_enabled_ = false;
            callback_queue_.clear();
            callback_exit_ = true;
        }
        queue_cv_.notify_all();
        callback_cv_.notify_all();
        if (callback_thread_.joinable()) {
            py::gil_scoped_release release;
            callback_thread_.join();
        }

        {
            std::lock_guard<std::mutex> lk(event_mutex_);
            on_data_ = py::object();
            on_status_ = py::object();
            on_info_ = py::object();
            direct_data_enabled_ = false;
            direct_status_enabled_ = false;
            direct_info_enabled_ = false;
        }

        std::lock_guard<std::mutex> g_lk(g_active_mutex);
        if (g_active_lidar == this) {
            g_active_lidar = nullptr;
        }
    }

    void set_on_frame(py::object callback) {
        if (!callback.is_none() && !py::isinstance<py::function>(callback)) {
            throw std::invalid_argument("callback must be callable or None");
        }
        std::lock_guard<std::mutex> lk(queue_mutex_);
        callback_ = callback.is_none() ? py::object() : callback;
        callback_enabled_ = static_cast<bool>(callback_);
    }

    void set_event_handlers(py::object on_data, py::object on_status, py::object on_info) {
        validate_event_handler(on_data, "on_data");
        validate_event_handler(on_status, "on_status");
        validate_event_handler(on_info, "on_info");

        std::lock_guard<std::mutex> lk(event_mutex_);
        on_data_ = on_data.is_none() ? py::object() : on_data;
        on_status_ = on_status.is_none() ? py::object() : on_status;
        on_info_ = on_info.is_none() ? py::object() : on_info;
        direct_data_enabled_ = static_cast<bool>(on_data_);
        direct_status_enabled_ = static_cast<bool>(on_status_);
        direct_info_enabled_ = static_cast<bool>(on_info_);
    }

    int send_sync() {
        return send_command_to_all(iTFS::packet::cmd_sync);
    }

    std::shared_ptr<Frame> read_frame(double timeout_sec, py::object device_index) {
        int index = parse_device_index(device_index);
        std::unique_lock<std::mutex> lk(queue_mutex_);
        bool ok = wait_for_event(lk, timeout_sec, [this, index] { return has_frame(index); });
        auto *queue = frame_queue_for(index);
        if (!ok || queue == nullptr || queue->empty()) {
            return nullptr;
        }
        auto frame = queue->front();
        queue->pop_front();
        return frame;
    }

    std::shared_ptr<Frame> latest_frame(py::object device_index) const {
        int index = parse_device_index(device_index);
        std::lock_guard<std::mutex> lk(queue_mutex_);
        if (index < 0) {
            return latest_;
        }
        auto it = latest_frames_.find(index);
        return it == latest_frames_.end() ? nullptr : it->second;
    }

    py::list devices() const {
        py::list out;
        std::lock_guard<std::mutex> lk(queue_mutex_);
        for (int i = 0; i < iTFS::max_device; i++) {
            auto it = devices_.find(i);
            if (it == devices_.end()) {
                continue;
            }
            py::dict d;
            const auto &dev = it->second;
            d["idx"] = dev.index;
            d["ip"] = dev.ip;
            d["port"] = dev.port;
            d["status"] = status_to_dict(dev.status);
            d["status_full"] = status_full_to_dict(dev.status_full);
            if (dev.info.sensor_sn != 0) {
                d["info"] = info_to_dict(dev.info);
            } else if (dev.info_v2.sensor_sn != 0) {
                d["info"] = info_v2_to_dict(dev.info_v2);
            } else {
                d["info"] = py::none();
            }
            out.append(d);
        }
        return out;
    }

    py::object read_status(double timeout_sec, py::object device_index) {
        int index = parse_device_index(device_index);
        std::unique_lock<std::mutex> lk(queue_mutex_);
        bool ok = wait_for_event(lk, timeout_sec, [this, index] { return has_status(index); });
        auto *queue = status_queue_for(index);
        if (!ok || queue == nullptr || queue->empty()) {
            return py::none();
        }
        auto event = queue->front();
        queue->pop_front();
        lk.unlock();
        return status_event_to_dict(event);
    }

    py::object read_info(double timeout_sec, py::object device_index) {
        int index = parse_device_index(device_index);
        std::unique_lock<std::mutex> lk(queue_mutex_);
        bool ok = wait_for_event(lk, timeout_sec, [this, index] { return has_info(index); });
        auto *queue = info_queue_for(index);
        if (!ok || queue == nullptr || queue->empty()) {
            return py::none();
        }
        auto event = queue->front();
        queue->pop_front();
        lk.unlock();
        return info_event_to_dict(event);
    }

    py::object latest_status(int device_index) const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        auto it = latest_status_.find(device_index);
        if (it == latest_status_.end()) {
            return py::none();
        }
        return status_event_to_dict(it->second);
    }

    py::object latest_status_full(int device_index) const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        auto it = latest_status_.find(device_index);
        if (it == latest_status_.end() || it->second.status_full.sensor_sn == 0) {
            return py::none();
        }
        return status_full_to_dict(it->second.status_full);
    }

    py::object latest_info(int device_index) const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        auto it = latest_info_.find(device_index);
        if (it == latest_info_.end()) {
            return py::none();
        }
        return info_event_to_dict(it->second);
    }

    void handle_image(iTFS::device_t *device) {
        // Data handler: forward the live device or queue a copied frame.
        if (direct_data_enabled_.load()) {
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
            }
            invoke_event(0, device);
            return;
        }

        auto frame = std::make_shared<Frame>();
        frame->device_index = device->idx;
        frame->ip = ip_to_string(device->ip);
        frame->port = device->port;
        frame->mode = device->data.mode;
        frame->frame_counter = device->data.frame;
        frame->frame_status = device->data.frame_status;
        frame->capture_row = device->data.capture_row > 0 ? device->data.capture_row : kRows;
        if (frame->capture_row > kRows) {
            frame->capture_row = kRows;
        }
        frame->status = device->status;
        frame->status_full = device->status_full;
        frame->info = device->info;
        frame->info_v2 = device->info_v2;
        uint8_t data_output = device->info.sensor_sn != 0 ? device->info.data_output : device->info_v2.data_output;
        frame->depth_on = (data_output & iTFS::packet::data_output_depth_mask) != 0 || frame->mode == 0;
        frame->intensity_on = (data_output & iTFS::packet::data_output_intensity_mask) != 0;
        frame->image.resize(static_cast<size_t>(2 * kRows) * kCols);
        std::memcpy(frame->image.data(), &device->data.img[0][0], frame->image.size() * sizeof(uint16_t));

        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            update_device_snapshot(device);
            latest_ = frame;
            latest_frames_[frame->device_index] = frame;
            frame_queue_.push_back(frame);
            frame_queues_[frame->device_index].push_back(frame);
            trim_frame_queue(frame_queue_, frame->device_index);
            while (frame_queues_[frame->device_index].size() > queue_size_) {
                frame_queues_[frame->device_index].pop_front();
            }
            if (callback_enabled_) {
                callback_queue_.push_back(frame);
                trim_frame_queue(callback_queue_, frame->device_index);
            }
        }
        queue_cv_.notify_all();
        callback_cv_.notify_one();
    }

    void handle_status(iTFS::device_t *device) {
        // Status handler: preserve the latest status and notify Python.
        if (direct_status_enabled_.load()) {
            StatusEvent event;
            event.device_index = device->idx;
            event.ip = ip_to_string(device->ip);
            event.port = device->port;
            event.status = device->status;
            event.status_full = device->status_full;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
                latest_status_[event.device_index] = event;
            }
            invoke_event(1, device);
            return;
        }

        StatusEvent event;
        event.device_index = device->idx;
        event.ip = ip_to_string(device->ip);
        event.port = device->port;
        event.status = device->status;
        event.status_full = device->status_full;
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            update_device_snapshot(device);
            latest_status_[event.device_index] = event;
            status_queue_.push_back(event);
            status_queues_[event.device_index].push_back(event);
            trim_event_queue(status_queue_, event.device_index);
            while (status_queues_[event.device_index].size() > queue_size_) {
                status_queues_[event.device_index].pop_front();
            }
        }
        queue_cv_.notify_all();
    }

    void handle_info(iTFS::device_t *device) {
        // Info handler: preserve the latest info and notify Python.
        if (direct_info_enabled_.load()) {
            InfoEvent event;
            event.device_index = device->idx;
            event.ip = ip_to_string(device->ip);
            event.port = device->port;
            event.info = device->info;
            event.info_v2 = device->info_v2;
            event.version = device->info.sensor_sn != 0 ? 1 :
                (device->info_v2.sensor_sn != 0 ? 2 : 0);
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
                latest_info_[event.device_index] = event;
            }
            invoke_event(2, device);
            return;
        }

        InfoEvent event;
        event.device_index = device->idx;
        event.ip = ip_to_string(device->ip);
        event.port = device->port;
        event.info = device->info;
        event.info_v2 = device->info_v2;
        event.version = device->info.sensor_sn != 0 ? 1 :
            (device->info_v2.sensor_sn != 0 ? 2 : 0);
        {
            std::lock_guard<std::mutex> lk(queue_mutex_);
            update_device_snapshot(device);
            latest_info_[event.device_index] = event;
            info_queue_.push_back(event);
            info_queues_[event.device_index].push_back(event);
            trim_event_queue(info_queue_, event.device_index);
            while (info_queues_[event.device_index].size() > queue_size_) {
                info_queues_[event.device_index].pop_front();
            }
        }
        queue_cv_.notify_all();
    }

    static void image_callback(iTFS::device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lidar != nullptr) {
            g_active_lidar->handle_image(device);
        }
    }

    static void status_callback(iTFS::device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lidar != nullptr) {
            g_active_lidar->handle_status(device);
        }
    }

    static void info_callback(iTFS::device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lidar != nullptr) {
            g_active_lidar->handle_info(device);
        }
    }

  private:
    int send_command_to_all(uint16_t command_id) {
        std::lock_guard<std::mutex> lk(close_mutex_);
        if (closed_ || !lidar_) {
            throw std::runtime_error("LiDAR is closed");
        }

        iTFS::packet::cmd_t command{};
        command.cmd_id = command_id;
        command.cmd_msg = 0;
        return lidar_->Send_cmd_to_all(&command);
    }

    static void validate_event_handler(const py::object &handler, const char *name) {
        if (!handler.is_none() && !py::isinstance<py::function>(handler)) {
            throw std::invalid_argument(std::string(name) + " must be callable or None");
        }
    }

    void invoke_event(int event_type, iTFS::device_t *device) {
        py::gil_scoped_acquire gil;
        py::object callback;
        {
            std::lock_guard<std::mutex> lk(event_mutex_);
            callback = event_type == 0 ? on_data_ : (event_type == 1 ? on_status_ : on_info_);
        }
        if (!callback) {
            return;
        }

        auto view = std::make_shared<Device>(device);
        try {
            callback(view);
        } catch (const py::error_already_set &e) {
            PyErr_WriteUnraisable(e.value().ptr());
        }
        view->invalidate();
    }

    void update_device_snapshot(const iTFS::device_t *device) {
        auto &snapshot = devices_[device->idx];
        snapshot.index = device->idx;
        snapshot.ip = ip_to_string(device->ip);
        snapshot.port = device->port;
        snapshot.status = device->status;
        snapshot.status_full = device->status_full;
        snapshot.info = device->info;
        snapshot.info_v2 = device->info_v2;
    }

    void trim_frame_queue(std::deque<std::shared_ptr<Frame>> &queue, int device_index) {
        size_t device_count = 0;
        for (const auto &frame : queue) {
            if (frame->device_index == device_index) {
                device_count++;
            }
        }
        if (device_count <= queue_size_) {
            return;
        }
        for (auto it = queue.begin(); it != queue.end(); ++it) {
            if ((*it)->device_index == device_index) {
                queue.erase(it);
                return;
            }
        }
    }

    template <typename Event>
    void trim_event_queue(std::deque<Event> &queue, int device_index) {
        size_t device_count = 0;
        for (const auto &event : queue) {
            if (event.device_index == device_index) {
                device_count++;
            }
        }
        if (device_count <= queue_size_) {
            return;
        }
        for (auto it = queue.begin(); it != queue.end(); ++it) {
            if (it->device_index == device_index) {
                queue.erase(it);
                return;
            }
        }
    }

    static int parse_device_index(const py::object &device_index) {
        if (device_index.is_none()) {
            return -1;
        }
        int index = py::cast<int>(device_index);
        if (index < 0 || index >= iTFS::max_device) {
            throw std::out_of_range("device_index is out of range");
        }
        return index;
    }

    bool has_frame(int index) const {
        if (index < 0) {
            return !frame_queue_.empty();
        }
        auto it = frame_queues_.find(index);
        return it != frame_queues_.end() && !it->second.empty();
    }

    bool has_status(int index) const {
        if (index < 0) {
            return !status_queue_.empty();
        }
        auto it = status_queues_.find(index);
        return it != status_queues_.end() && !it->second.empty();
    }

    bool has_info(int index) const {
        if (index < 0) {
            return !info_queue_.empty();
        }
        auto it = info_queues_.find(index);
        return it != info_queues_.end() && !it->second.empty();
    }

    std::deque<std::shared_ptr<Frame>> *frame_queue_for(int index) {
        if (index < 0) {
            return &frame_queue_;
        }
        auto it = frame_queues_.find(index);
        return it == frame_queues_.end() ? nullptr : &it->second;
    }

    std::deque<StatusEvent> *status_queue_for(int index) {
        if (index < 0) {
            return &status_queue_;
        }
        auto it = status_queues_.find(index);
        return it == status_queues_.end() ? nullptr : &it->second;
    }

    std::deque<InfoEvent> *info_queue_for(int index) {
        if (index < 0) {
            return &info_queue_;
        }
        auto it = info_queues_.find(index);
        return it == info_queues_.end() ? nullptr : &it->second;
    }

    template <typename Predicate>
    bool wait_for_event(std::unique_lock<std::mutex> &lk, double timeout_sec, Predicate pred) {
        if (timeout_sec < 0) {
            queue_cv_.wait(lk, [this, &pred] { return closed_ || pred(); });
            return pred();
        }
        return queue_cv_.wait_for(
            lk,
            std::chrono::duration<double>(timeout_sec),
            [this, &pred] { return closed_ || pred(); });
    }

    void callback_loop() {
        while (true) {
            std::shared_ptr<Frame> frame;
            py::object callback;
            {
                std::unique_lock<std::mutex> lk(queue_mutex_);
                callback_cv_.wait(lk, [this] { return callback_exit_ || !callback_queue_.empty(); });
                if (callback_exit_ && callback_queue_.empty()) {
                    return;
                }
                frame = callback_queue_.front();
                callback_queue_.pop_front();
            }

            try {
                py::gil_scoped_acquire gil;
                {
                    std::lock_guard<std::mutex> lk(queue_mutex_);
                    callback = callback_;
                }
                if (callback) {
                    callback(frame);
                }
            } catch (const py::error_already_set &e) {
                py::gil_scoped_acquire gil;
                PyErr_WriteUnraisable(e.value().ptr());
            }
        }
    }

    std::unique_ptr<iTFS::LiDAR> lidar_;
    std::array<uint8_t, 4> broadcast_ip_{};
    std::array<uint8_t, 4> listening_ip_{};
    size_t queue_size_;

    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::condition_variable callback_cv_;
    std::deque<std::shared_ptr<Frame>> frame_queue_;
    std::deque<StatusEvent> status_queue_;
    std::deque<InfoEvent> info_queue_;
    std::unordered_map<int, std::deque<std::shared_ptr<Frame>>> frame_queues_;
    std::unordered_map<int, std::deque<StatusEvent>> status_queues_;
    std::unordered_map<int, std::deque<InfoEvent>> info_queues_;
    std::deque<std::shared_ptr<Frame>> callback_queue_;
    std::shared_ptr<Frame> latest_;
    std::unordered_map<int, std::shared_ptr<Frame>> latest_frames_;
    std::unordered_map<int, StatusEvent> latest_status_;
    std::unordered_map<int, InfoEvent> latest_info_;
    std::unordered_map<int, DeviceSnapshot> devices_;
    py::object callback_;
    bool callback_enabled_ = false;

    std::mutex event_mutex_;
    py::object on_data_;
    py::object on_status_;
    py::object on_info_;
    std::atomic<bool> direct_data_enabled_{false};
    std::atomic<bool> direct_status_enabled_{false};
    std::atomic<bool> direct_info_enabled_{false};

    std::thread callback_thread_;
    bool callback_exit_ = false;

    mutable std::mutex close_mutex_;
    bool closed_ = false;
};

py::dict constants_dict() {
    py::dict d;
    d["MAX_ROW"] = iTFS::max_row;
    d["GRAY_ROW"] = iTFS::gray_row;
    d["MAX_COL"] = iTFS::max_col;
    d["DATA_OUTPUT_DEPTH_ON"] = iTFS::packet::data_output_depth_on;
    d["DATA_OUTPUT_INTENSITY_ON"] = iTFS::packet::data_output_intensity_on;
    d["DATA_OUTPUT_STATUS_FULL"] = iTFS::packet::data_output_status_full;
    return d;
}

} // namespace

PYBIND11_MODULE(_itfs, m) {
    if (iTFS::version() != 0) {
        throw std::runtime_error("iTFS library and packet header versions do not match");
    }

    m.doc() = "Native Python bindings for the iTFS LiDAR C++ SDK";

    // Python-owned image copy overwritten by each received data event.
    py::class_<Img, std::shared_ptr<Img>>(m, "Img")
        .def(py::init<>())
        .def_property_readonly("idx", [](const Img &f) { return f.device_index; })
        .def_property_readonly("ip", [](const Img &f) { return f.ip; })
        .def_property_readonly("port", [](const Img &f) { return f.port; })
        .def_property_readonly("mode", [](const Img &f) { return f.image.mode; })
        .def_property_readonly("frame", [](const Img &f) { return f.image.frame; })
        .def_property_readonly("frame_status", [](const Img &f) { return f.image.frame_status; })
        .def_property_readonly("capture_row", &copied_rows)
        .def_property_readonly("data_output", [](const Img &f) { return f.data_output; })
        .def_property_readonly("depth_on", [](const Img &f) {
            return (f.data_output & iTFS::packet::data_output_depth_mask) != 0 || f.image.mode == 0;
        })
        .def_property_readonly("intensity_on", [](const Img &f) {
            return (f.data_output & iTFS::packet::data_output_intensity_mask) != 0 && f.image.mode != 0;
        })
        .def_property_readonly("raw_image", [](py::object self) {
            return copied_image_to_numpy(self.cast<Img &>(), 0, 2 * kRows, self);
        })
        .def_property_readonly("image", [](py::object self) {
            return copied_depth_to_numpy(self.cast<Img &>(), self);
        })
        .def_property_readonly("depth", [](py::object self) {
            return copied_depth_to_numpy(self.cast<Img &>(), self);
        })
        .def_property_readonly("intensity", [](py::object self) {
            return copied_intensity_to_numpy(self.cast<Img &>(), self);
        })
        .def_property_readonly("depth_display", [](const Img &f) {
            return copied_display_to_numpy(f, 0);
        })
        .def_property_readonly("intensity_display", [](const Img &f) -> py::object {
            bool intensity_on =
                (f.data_output & iTFS::packet::data_output_intensity_mask) != 0 && f.image.mode != 0;
            if (!intensity_on) {
                return py::none();
            }
            return copied_display_to_numpy(f, kRows);
        })
        .def("point_cloud", &copied_point_cloud_to_numpy, py::arg("intrinsic"));

    // Live device view passed only while a direct event callback is running.
    py::class_<Device, std::shared_ptr<Device>>(m, "Device")
        .def_property_readonly("idx", &Device::device_index)
        .def_property_readonly("ip", &Device::ip)
        .def_property_readonly("port", &Device::port)
        .def_property_readonly("status", &Device::status)
        .def_property_readonly("info", &Device::info)
        .def("copy_image", &Device::copy_image, py::arg("target"));

    // Native receive runtime and its public Python API.
    py::class_<LiDAR>(m, "LiDAR")
        .def(py::init<size_t, py::object, py::object, uint16_t>(),
             py::arg("queue_size") = 2,
             py::arg("broadcast_ip") = py::none(),
             py::arg("listening_ip") = py::none(),
             py::arg("listening_port") = iTFS::user_data_port)
        .def("close", &LiDAR::close)
        .def("__enter__", [](LiDAR &self) -> LiDAR & { return self; })
        .def("__exit__", [](LiDAR &self, py::object, py::object, py::object) { self.close(); })
        .def_property_readonly("ready", &LiDAR::ready)
        .def_property_readonly("device_count", &LiDAR::device_count)
        .def("devices", &LiDAR::devices)
        .def("set_event_handlers", &LiDAR::set_event_handlers,
             py::arg("on_data") = py::none(),
             py::arg("on_status") = py::none(),
             py::arg("on_info") = py::none())
        .def("send_sync", &LiDAR::send_sync)
        .def("read_status", &LiDAR::read_status, py::arg("timeout") = 1.0, py::arg("idx") = py::none())
        .def("read_info", &LiDAR::read_info, py::arg("timeout") = 1.0, py::arg("idx") = py::none())
        .def("latest_status", &LiDAR::latest_status, py::arg("idx") = 0)
        .def("latest_status_full", &LiDAR::latest_status_full, py::arg("idx") = 0)
        .def("latest_info", &LiDAR::latest_info, py::arg("idx") = 0);

    m.attr("max_device") = iTFS::max_device;
    m.attr("constants") = constants_dict();
}
