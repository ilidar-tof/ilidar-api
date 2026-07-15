/**
 * @file ilidar_lite_pybind.cpp
 * @brief Python bindings for the iTFS-LITE C++ SDK
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

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
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

#include "ilidar_lite.hpp"

namespace py = pybind11;

namespace {

constexpr int kRows = iTFS::lite_max_row;
constexpr int kCols = iTFS::lite_max_col;

// Native event and image snapshots stored independently of SDK callback memory.
struct ImagePlane {
    int class_id = -1;
    std::string name;
    bool is_u8 = false;
    int rows = kRows;
    int cols = kCols;
    std::vector<uint8_t> bytes;
};

struct Frame {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    uint8_t mode = 0;
    uint8_t frame_counter = 0;
    int frame_status = 0;
    int capture_row = kRows;
    bool depth_on = false;
    bool amplitude_on = false;
    bool intensity_on = false;
    bool confidence_on = false;
    bool xyz_on = false;
    iTFS::packet::status_v3_t status{};
    iTFS::packet::info_v3_t info{};
    bool reconstruction_map_valid = false;
    std::vector<float> reconstruction_map;
    std::unordered_map<int, ImagePlane> planes;
};

struct StatusEvent {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    iTFS::packet::status_v3_t status{};
};

struct InfoEvent {
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    iTFS::packet::info_v3_t info{};
};

struct DeviceSnapshot {
    int index = -1;
    std::string ip;
    uint16_t port = 0;
    iTFS::packet::status_v3_t status{};
    iTFS::packet::info_v3_t info{};
};

struct LiteImgCpy {
    LiteImgCpy() {
        for (int i = 0; i < iTFS::lite_img_class_count; i++) {
            image.img_offset[i] = -1;
        }
    }

    iTFS::lite_img_cpy_t image{};
    int device_index = -1;
    std::string ip;
    uint16_t port = 0;
    int capture_row = kRows;
    uint16_t data_output = 0;
    std::shared_ptr<std::vector<float>> reconstruction_map;
};

class LITE;

std::mutex g_active_mutex;
LITE *g_active_lite = nullptr;

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

std::string class_name(int class_id) {
    switch (class_id) {
    case iTFS::lite_img_depth:
        return "depth";
    case iTFS::lite_img_amplitude:
        return "amplitude";
    case iTFS::lite_img_intensity:
        return "intensity";
    case iTFS::lite_img_confidence:
        return "confidence";
    case iTFS::lite_img_point_x:
        return "point_x";
    case iTFS::lite_img_point_y:
        return "point_y";
    default:
        return "unknown";
    }
}

// LITE image layout and conversion helpers.
bool class_is_u8(const iTFS::lite_device_t *device, int class_id) {
    uint16_t data_output = device->info_v3.data_output;
    uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
    uint16_t amplitude_mode =
        (data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >>
        iTFS::packet::info_v3_data_output_amplitude_pos;
    uint16_t intensity_mode =
        (data_output & iTFS::packet::info_v3_data_output_intensity_mask) >>
        iTFS::packet::info_v3_data_output_intensity_pos;
    uint16_t confidence_mode =
        (data_output & iTFS::packet::info_v3_data_output_confidence_mask) >>
        iTFS::packet::info_v3_data_output_confidence_pos;

    if (class_id == iTFS::lite_img_depth) {
        return depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit ||
               depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit ||
               depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;
    }
    if (class_id == iTFS::lite_img_point_x || class_id == iTFS::lite_img_point_y) {
        return depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;
    }
    if (class_id == iTFS::lite_img_amplitude) {
        return amplitude_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
               amplitude_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    }
    if (class_id == iTFS::lite_img_intensity) {
        return intensity_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
               intensity_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    }
    if (class_id == iTFS::lite_img_confidence) {
        return confidence_mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;
    }
    return false;
}

py::object image_to_numpy(const Frame &frame, int class_id) {
    auto it = frame.planes.find(class_id);
    if (it == frame.planes.end()) {
        return py::none();
    }

    const ImagePlane &plane = it->second;
    if (plane.is_u8) {
        py::array_t<uint8_t> arr({plane.rows, plane.cols});
        std::memcpy(arr.mutable_data(), plane.bytes.data(), plane.bytes.size());
        return arr;
    }

    py::array_t<uint16_t> arr({plane.rows, plane.cols});
    std::memcpy(arr.mutable_data(), plane.bytes.data(), plane.bytes.size());
    return arr;
}

const ImagePlane *find_plane(const Frame &frame, int class_id) {
    auto it = frame.planes.find(class_id);
    if (it == frame.planes.end()) {
        return nullptr;
    }
    return &it->second;
}

float depth_log8_to_m(uint8_t value) {
    return static_cast<float>(iTFS::depth_log8_lut_lite::decode_mm(value)) * 0.001f;
}

float read_u16_as_m(const ImagePlane &plane, int idx, float scale) {
    const uint16_t *data = reinterpret_cast<const uint16_t *>(plane.bytes.data());
    return static_cast<float>(data[idx]) * scale;
}

float read_s16_as_m(const ImagePlane &plane, int idx, float scale) {
    const int16_t *data = reinterpret_cast<const int16_t *>(plane.bytes.data());
    return static_cast<float>(data[idx]) * scale;
}

float read_u8_as_m(const ImagePlane &plane, int idx, float scale) {
    const uint8_t *data = plane.bytes.data();
    return static_cast<float>(data[idx]) * scale;
}

float read_s8_as_m(const ImagePlane &plane, int idx, float scale) {
    const int8_t *data = reinterpret_cast<const int8_t *>(plane.bytes.data());
    return static_cast<float>(data[idx]) * scale;
}

py::object point_cloud_to_numpy(const Frame &frame) {
    constexpr float depth_q16_max_m = 7.49481145f;
    constexpr float depth_raw_q16_to_m = depth_q16_max_m / 65535.0f;
    constexpr float depth_lin8_to_m = depth_q16_max_m / 255.0f;
    constexpr float xyz_raw_q15_to_m = depth_q16_max_m / 32767.0f;
    constexpr float xyz_lin8_to_m = depth_q16_max_m / 127.0f;

    uint16_t data_output = frame.info.data_output;
    uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
    const ImagePlane *depth = find_plane(frame, iTFS::lite_img_depth);

    std::vector<float> points;
    points.reserve(static_cast<size_t>(frame.capture_row) * kCols * 3);

    if (frame.depth_on && depth != nullptr && frame.reconstruction_map_valid) {
        for (int r = 0; r < frame.capture_row; r++) {
            for (int c = 0; c < kCols; c++) {
                int idx = r * kCols + c;
                float z_m = 0.0f;
                if (depth_mode == iTFS::packet::info_v3_data_output_depth_mm_16bit) {
                    z_m = read_u16_as_m(*depth, idx, 0.001f);
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16) {
                    z_m = read_u16_as_m(*depth, idx, depth_raw_q16_to_m);
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit) {
                    z_m = read_u8_as_m(*depth, idx, depth_lin8_to_m);
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                    z_m = depth_log8_to_m(depth->bytes[idx]);
                } else {
                    continue;
                }
                size_t map_idx = static_cast<size_t>(idx) * 3;
                float rx = frame.reconstruction_map[map_idx + 0];
                float ry = frame.reconstruction_map[map_idx + 1];
                float rz = frame.reconstruction_map[map_idx + 2];
                points.push_back(z_m * rz);
                points.push_back(-z_m * rx);
                points.push_back(-z_m * ry);
            }
        }
    } else if (frame.xyz_on && depth != nullptr) {
        const ImagePlane *point_x = find_plane(frame, iTFS::lite_img_point_x);
        const ImagePlane *point_y = find_plane(frame, iTFS::lite_img_point_y);
        if (point_x == nullptr || point_y == nullptr) {
            return py::none();
        }

        for (int r = 0; r < frame.capture_row; r++) {
            for (int c = 0; c < kCols; c++) {
                int idx = r * kCols + c;
                float x_m = 0.0f;
                float y_m = 0.0f;
                float z_m = 0.0f;

                if (depth_mode == iTFS::packet::info_v3_data_output_xyz_mm_16bit) {
                    x_m = read_s16_as_m(*point_x, idx, 0.001f);
                    y_m = read_s16_as_m(*point_y, idx, 0.001f);
                    z_m = read_u16_as_m(*depth, idx, 0.001f);
                } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
                    x_m = read_s16_as_m(*point_x, idx, xyz_raw_q15_to_m);
                    y_m = read_s16_as_m(*point_y, idx, xyz_raw_q15_to_m);
                    z_m = read_u16_as_m(*depth, idx, depth_raw_q16_to_m);
                } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit) {
                    x_m = read_s8_as_m(*point_x, idx, xyz_lin8_to_m);
                    y_m = read_s8_as_m(*point_y, idx, xyz_lin8_to_m);
                    z_m = read_u8_as_m(*depth, idx, depth_lin8_to_m);
                } else {
                    continue;
                }
                points.push_back(z_m);
                points.push_back(-x_m);
                points.push_back(-y_m);
            }
        }
    } else {
        return py::none();
    }

    py::array_t<float> arr({static_cast<py::ssize_t>(points.size() / 3), static_cast<py::ssize_t>(3)});
    std::memcpy(arr.mutable_data(), points.data(), points.size() * sizeof(float));
    return arr;
}

py::dict status_to_dict(const iTFS::packet::status_v3_t &s) {
    py::dict d;
    d["capture_mode"] = s.capture_mode;
    d["capture_frame"] = s.capture_frame;
    d["sensor_sn"] = s.sensor_sn;
    d["sensor_time_us"] = iTFS::packet::get_sensor_time_in_us(s.sensor_time_th, s.sensor_time_tl);
    d["sensor_frame_status"] = s.sensor_frame_status;
    d["temp_rx_c"] = s.sensor_temp_rx * 0.01;
    d["temp_tx_c"] = s.sensor_temp_tx * 0.01;
    d["temp_core_c"] = s.sensor_temp_core * 0.01;
    d["usb_level_v"] = s.sensor_usb_level * 0.01;
    d["power_level_v"] = py::make_tuple(s.sensor_power_level[0] * 0.01, s.sensor_power_level[1] * 0.01);
    d["cpu_usage_percent"] = s.sensor_cpu_usage * 0.01;
    d["ram_usage_percent"] = s.sensor_ram_usage * 0.01;
    d["warning"] = s.sensor_warning;
    d["exposure_us"] = s.sensor_exposure;
    d["frame_drop_count"] = s.sensor_frame_drop_count;
    d["udp_rx_drop_count"] = s.sensor_udp_rx_drop_count;
    d["udp_tx_drop_count"] = s.sensor_udp_tx_drop_count;
    return d;
}

py::dict status_event_to_dict(const StatusEvent &event) {
    py::dict d = status_to_dict(event.status);
    d["idx"] = event.device_index;
    d["ip"] = event.ip;
    d["port"] = event.port;
    return d;
}

std::string fixed_char_string(const char *data, size_t size) {
    size_t len = 0;
    while (len < size && data[len] != '\0') {
        len++;
    }
    return std::string(data, len);
}

py::dict info_to_dict(const iTFS::packet::info_v3_t &info) {
    py::dict d;
    d["sensor_sn"] = info.sensor_sn;
    d["sensor_hw_id"] = fixed_char_string(reinterpret_cast<const char *>(info.sensor_hw_id), 64);
    d["sensor_fw_ver"] = py::make_tuple(info.sensor_fw_ver[2], info.sensor_fw_ver[1], info.sensor_fw_ver[0]);
    d["sensor_fw_date"] = fixed_char_string(info.sensor_fw_date, 12);
    d["sensor_fw_time"] = fixed_char_string(info.sensor_fw_time, 9);
    d["capture_mode"] = info.capture_mode;
    d["capture_row"] = info.capture_row;
    d["capture_shutter"] = py::make_tuple(info.capture_shutter[0], info.capture_shutter[1], info.capture_shutter[2]);
    d["capture_period_ns"] = info.capture_period_ns;
    d["data_output"] = info.data_output;
    d["data_sensor_ip"] = ip_to_string(info.data_sensor_ip);
    d["data_dest_ip"] = ip_to_string(info.data_dest_ip);
    d["data_port"] = info.data_port;
    d["dhcp"] = static_cast<bool>(info.data_dhcp_ctrl);
    d["lock"] = info.lock;
    return d;
}

py::dict info_event_to_dict(const InfoEvent &event) {
    py::dict d = info_to_dict(event.info);
    d["idx"] = event.device_index;
    d["ip"] = event.ip;
    d["port"] = event.port;
    return d;
}

bool image_class_is_u8(uint16_t data_output, int class_id) {
    uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
    uint16_t amplitude_mode =
        (data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >>
        iTFS::packet::info_v3_data_output_amplitude_pos;
    uint16_t intensity_mode =
        (data_output & iTFS::packet::info_v3_data_output_intensity_mask) >>
        iTFS::packet::info_v3_data_output_intensity_pos;
    uint16_t confidence_mode =
        (data_output & iTFS::packet::info_v3_data_output_confidence_mask) >>
        iTFS::packet::info_v3_data_output_confidence_pos;

    if (class_id == iTFS::lite_img_depth ||
        class_id == iTFS::lite_img_point_x ||
        class_id == iTFS::lite_img_point_y) {
        return depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit ||
               depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit ||
               depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;
    }
    if (class_id == iTFS::lite_img_amplitude) {
        return amplitude_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
               amplitude_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    }
    if (class_id == iTFS::lite_img_intensity) {
        return intensity_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
               intensity_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
    }
    if (class_id == iTFS::lite_img_confidence) {
        return confidence_mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;
    }
    return false;
}

py::object copied_image_to_numpy(LiteImgCpy &copy, int class_id, py::handle owner) {
    if (class_id < 0 || class_id >= iTFS::lite_img_class_count ||
        copy.image.img_offset[class_id] < 0) {
        return py::none();
    }

    int slot = copy.image.img_offset[class_id] / iTFS::lite_max_row;
    if (image_class_is_u8(copy.data_output, class_id)) {
        return py::array(
            py::dtype::of<uint8_t>(),
            {copy.capture_row, kCols},
            {static_cast<py::ssize_t>(kCols * sizeof(uint8_t)), static_cast<py::ssize_t>(sizeof(uint8_t))},
            &copy.image.data[slot].u8[0][0],
            owner);
    }
    return py::array(
        py::dtype::of<uint16_t>(),
        {copy.capture_row, kCols},
        {static_cast<py::ssize_t>(kCols * sizeof(uint16_t)), static_cast<py::ssize_t>(sizeof(uint16_t))},
        &copy.image.data[slot].u16[0][0],
        owner);
}

uint8_t display_u8(double value) {
    long rounded = std::lround(value);
    if (rounded < 0) {
        return 0;
    }
    if (rounded > 255) {
        return 255;
    }
    return static_cast<uint8_t>(rounded);
}

const iTFS::lite_img_slot_t *copied_slot(const LiteImgCpy &copy, int class_id);

py::object copied_display_to_numpy(const LiteImgCpy &copy, int class_id) {
    const auto *slot = copied_slot(copy, class_id);
    if (slot == nullptr) {
        return py::none();
    }

    py::array_t<uint8_t> output({copy.capture_row, kCols});
    uint8_t *dst = output.mutable_data();
    size_t pixels = static_cast<size_t>(copy.capture_row) * kCols;
    const uint8_t *src_u8 = &slot->u8[0][0];
    const uint16_t *src_u16 = &slot->u16[0][0];

    uint16_t mode = 0;
    if (class_id == iTFS::lite_img_depth) {
        mode = copy.data_output & iTFS::packet::info_v3_data_output_depth_mask;
        for (size_t i = 0; i < pixels; i++) {
            if (mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                dst[i] = display_u8(
                    iTFS::depth_log8_lut_lite::decode_mm(src_u8[i]) * 255.0 / 7494.0);
            } else if (mode == iTFS::packet::info_v3_data_output_depth_raw_q16 ||
                       mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
                dst[i] = display_u8(src_u16[i] * 255.0 / 65535.0);
            } else if (image_class_is_u8(copy.data_output, class_id)) {
                dst[i] = src_u8[i];
            } else {
                dst[i] = display_u8(src_u16[i] * 255.0 / 7494.0);
            }
        }
        return output;
    }

    if (class_id == iTFS::lite_img_amplitude) {
        mode = (copy.data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >>
               iTFS::packet::info_v3_data_output_amplitude_pos;
    } else if (class_id == iTFS::lite_img_intensity) {
        mode = (copy.data_output & iTFS::packet::info_v3_data_output_intensity_mask) >>
               iTFS::packet::info_v3_data_output_intensity_pos;
    } else if (class_id == iTFS::lite_img_confidence) {
        mode = (copy.data_output & iTFS::packet::info_v3_data_output_confidence_mask) >>
               iTFS::packet::info_v3_data_output_confidence_pos;
        bool mask1 = mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;
        for (size_t i = 0; i < pixels; i++) {
            dst[i] = mask1 ? display_u8(src_u8[i] * 255.0) : display_u8(src_u16[i] * 255.0 / 65535.0);
        }
        return output;
    } else {
        return py::none();
    }

    bool is_u8 = image_class_is_u8(copy.data_output, class_id);
    bool linear8 = mode == iTFS::packet::info_v3_data_output_stream_lin_8bit;
    for (size_t i = 0; i < pixels; i++) {
        if (is_u8) {
            dst[i] = linear8 ? display_u8(src_u8[i] * 255.0 / 0x3F) : src_u8[i];
        } else {
            dst[i] = display_u8(src_u16[i] * 255.0 / 0x3FFF);
        }
    }
    return output;
}

const iTFS::lite_img_slot_t *copied_slot(const LiteImgCpy &copy, int class_id) {
    if (class_id < 0 || class_id >= iTFS::lite_img_class_count ||
        copy.image.img_offset[class_id] < 0) {
        return nullptr;
    }
    return &copy.image.data[copy.image.img_offset[class_id] / iTFS::lite_max_row];
}

py::object copied_point_cloud_to_numpy(const LiteImgCpy &copy) {
    constexpr float depth_q16_max_m = 7.49481145f;
    constexpr float depth_raw_q16_to_m = depth_q16_max_m / 65535.0f;
    constexpr float depth_lin8_to_m = depth_q16_max_m / 255.0f;
    constexpr float xyz_raw_q15_to_m = depth_q16_max_m / 32767.0f;
    constexpr float xyz_lin8_to_m = depth_q16_max_m / 127.0f;

    uint16_t depth_mode = copy.data_output & iTFS::packet::info_v3_data_output_depth_mask;
    const auto *depth = copied_slot(copy, iTFS::lite_img_depth);
    if (depth == nullptr) {
        return py::none();
    }
    const uint16_t *depth_u16 = &depth->u16[0][0];
    const uint8_t *depth_u8 = &depth->u8[0][0];

    std::vector<float> points;
    points.reserve(static_cast<size_t>(copy.capture_row) * kCols * 3);

    for (int r = 0; r < copy.capture_row; r++) {
        for (int c = 0; c < kCols; c++) {
            int idx = r * kCols + c;
            float x_m = 0.0f;
            float y_m = 0.0f;
            float z_m = 0.0f;

            if (copy.image.depth_on && copy.reconstruction_map) {
                if (depth_mode == iTFS::packet::info_v3_data_output_depth_mm_16bit) {
                    z_m = depth_u16[idx] * 0.001f;
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16) {
                    z_m = depth_u16[idx] * depth_raw_q16_to_m;
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit) {
                    z_m = depth_u8[idx] * depth_lin8_to_m;
                } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                    z_m = depth_log8_to_m(depth_u8[idx]);
                } else {
                    continue;
                }
                size_t map_idx = static_cast<size_t>(idx) * 3;
                x_m = z_m * (*copy.reconstruction_map)[map_idx + 2];
                y_m = -z_m * (*copy.reconstruction_map)[map_idx + 0];
                z_m = -z_m * (*copy.reconstruction_map)[map_idx + 1];
            } else if (copy.image.xyz_on) {
                const auto *point_x = copied_slot(copy, iTFS::lite_img_point_x);
                const auto *point_y = copied_slot(copy, iTFS::lite_img_point_y);
                if (point_x == nullptr || point_y == nullptr) {
                    return py::none();
                }
                const int16_t *point_x_s16 = &point_x->s16[0][0];
                const int16_t *point_y_s16 = &point_y->s16[0][0];
                const int8_t *point_x_s8 = &point_x->s8[0][0];
                const int8_t *point_y_s8 = &point_y->s8[0][0];
                float sensor_x = 0.0f;
                float sensor_y = 0.0f;
                if (depth_mode == iTFS::packet::info_v3_data_output_xyz_mm_16bit) {
                    sensor_x = point_x_s16[idx] * 0.001f;
                    sensor_y = point_y_s16[idx] * 0.001f;
                    z_m = depth_u16[idx] * 0.001f;
                } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
                    sensor_x = point_x_s16[idx] * xyz_raw_q15_to_m;
                    sensor_y = point_y_s16[idx] * xyz_raw_q15_to_m;
                    z_m = depth_u16[idx] * depth_raw_q16_to_m;
                } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit) {
                    sensor_x = point_x_s8[idx] * xyz_lin8_to_m;
                    sensor_y = point_y_s8[idx] * xyz_lin8_to_m;
                    z_m = depth_u8[idx] * depth_lin8_to_m;
                } else {
                    continue;
                }
                x_m = z_m;
                y_m = -sensor_x;
                z_m = -sensor_y;
            } else {
                return py::none();
            }

            points.push_back(x_m);
            points.push_back(y_m);
            points.push_back(z_m);
        }
    }

    py::array_t<float> arr({static_cast<py::ssize_t>(points.size() / 3), static_cast<py::ssize_t>(3)});
    std::memcpy(arr.mutable_data(), points.data(), points.size() * sizeof(float));
    return arr;
}

// Callback-only view of the SDK-owned device structure.
class LiteDevice {
  public:
    LiteDevice(iTFS::lite_device_t *device, std::shared_ptr<std::vector<float>> reconstruction_map)
        : device_(device), reconstruction_map_(std::move(reconstruction_map)) {}

    void invalidate() { device_ = nullptr; }

    int device_index() const { return device()->idx; }
    std::string ip() const { return ip_to_string(device()->ip); }
    uint16_t port() const { return device()->port; }

    py::dict status() const {
        StatusEvent event;
        event.device_index = device()->idx;
        event.ip = ip_to_string(device()->ip);
        event.port = device()->port;
        event.status = device()->status_v3;
        return status_event_to_dict(event);
    }

    py::dict info() const {
        InfoEvent event;
        event.device_index = device()->idx;
        event.ip = ip_to_string(device()->ip);
        event.port = device()->port;
        event.info = device()->info_v3;
        return info_event_to_dict(event);
    }

    void copy_image(LiteImgCpy &copy) const {
        // Deep-copy only active image slots, matching the C++ OpenCV handler.
        const auto *src = &device()->data;
        iTFS::copy_img_metadata(&copy.image, const_cast<iTFS::lite_img_t *>(src));
        size_t class_count = std::min<size_t>(copy.image.data_class_count, iTFS::lite_img_class_count);
        std::memcpy(copy.image.data, src->data, sizeof(iTFS::lite_img_slot_t) * class_count);
        copy.device_index = device()->idx;
        copy.ip = ip_to_string(device()->ip);
        copy.port = device()->port;
        copy.capture_row = src->capture_row > 0 ? std::min(src->capture_row, kRows) : kRows;
        copy.data_output = device()->info_v3.data_output;
        copy.reconstruction_map = reconstruction_map_;
    }

  private:
    iTFS::lite_device_t *device() const {
        if (device_ == nullptr) {
            throw std::runtime_error("LiteDevice is valid only while its event callback is running");
        }
        return device_;
    }

    iTFS::lite_device_t *device_;
    std::shared_ptr<std::vector<float>> reconstruction_map_;
};

// Owns the native SDK runtime and exposes queued and direct callback APIs.
class LITE {
  public:
    LITE(size_t queue_size,
         py::object broadcast_ip,
         py::object listening_ip,
         uint16_t listening_port)
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
            if (g_active_lite != nullptr) {
                throw std::runtime_error("only one ilidar_lite.LITE instance can be active at a time");
            }
            g_active_lite = this;
        }

        callback_thread_ = std::thread([this] { callback_loop(); });

        try {
            py::gil_scoped_release release;
            lite_ = std::make_unique<iTFS::LITE>(
                &LITE::image_callback,
                &LITE::status_callback,
                &LITE::info_callback,
                broadcast_ptr,
                listen_ptr,
                listening_port);
        } catch (...) {
            close();
            throw;
        }
    }

    ~LITE() {
        close();
    }

    bool ready() const {
        return lite_ && lite_->Ready();
    }

    int device_count() const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        return static_cast<int>(devices_.size());
    }

    void close() {
        std::unique_ptr<iTFS::LITE> local_lite;
        {
            std::lock_guard<std::mutex> lk(close_mutex_);
            if (closed_) {
                return;
            }
            closed_ = true;
            local_lite = std::move(lite_);
        }

        if (local_lite) {
            py::gil_scoped_release release;
            local_lite->Try_exit();
            local_lite->Join();
            local_lite.reset();
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
        if (g_active_lite == this) {
            g_active_lite = nullptr;
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
            d["info"] = info_to_dict(dev.info);
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

    py::object latest_info(int device_index) const {
        std::lock_guard<std::mutex> lk(queue_mutex_);
        auto it = latest_info_.find(device_index);
        if (it == latest_info_.end()) {
            return py::none();
        }
        return info_event_to_dict(it->second);
    }

    void handle_image(iTFS::lite_device_t *device) {
        // Data handler: forward the live device or queue a copied frame.
        if (direct_data_enabled_.load()) {
            std::shared_ptr<std::vector<float>> reconstruction_map;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
                reconstruction_map = get_reconstruction_map(device);
            }
            invoke_event(0, device, std::move(reconstruction_map));
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
        frame->depth_on = device->data.depth_on;
        frame->amplitude_on = device->data.amplitude_on;
        frame->intensity_on = device->data.intensity_on;
        frame->confidence_on = device->data.confidence_on;
        frame->xyz_on = device->data.xyz_on;
        frame->status = device->status_v3;
        frame->info = device->info_v3;
        frame->reconstruction_map_valid = device->reconstruction_map.valid;
        if (frame->reconstruction_map_valid) {
            frame->reconstruction_map.resize(static_cast<size_t>(kRows) * kCols * 3);
            for (int r = 0; r < kRows; r++) {
                for (int c = 0; c < kCols; c++) {
                    size_t idx = static_cast<size_t>(r * kCols + c) * 3;
                    frame->reconstruction_map[idx + 0] = device->reconstruction_map.dir[0][r][c];
                    frame->reconstruction_map[idx + 1] = device->reconstruction_map.dir[1][r][c];
                    frame->reconstruction_map[idx + 2] = device->reconstruction_map.dir[2][r][c];
                }
            }
        }

        for (int slot = 0; slot < device->data.data_class_count; slot++) {
            int class_id = device->data.data_class[slot];
            if (class_id < 0 || class_id >= iTFS::lite_img_class_count) {
                continue;
            }
            if (device->data.img_offset[class_id] < 0) {
                continue;
            }

            ImagePlane plane;
            plane.class_id = class_id;
            plane.name = class_name(class_id);
            plane.is_u8 = class_is_u8(device, class_id);
            plane.rows = frame->capture_row;
            plane.cols = kCols;
            size_t elem_size = plane.is_u8 ? sizeof(uint8_t) : sizeof(uint16_t);
            size_t byte_count = static_cast<size_t>(plane.rows) * plane.cols * elem_size;
            plane.bytes.resize(byte_count);

            int data_slot = device->data.img_offset[class_id] / iTFS::lite_max_row;
            if (plane.is_u8) {
                std::memcpy(plane.bytes.data(), &device->data.data[data_slot].u8[0][0], byte_count);
            } else {
                std::memcpy(plane.bytes.data(), &device->data.data[data_slot].u16[0][0], byte_count);
            }
            frame->planes[class_id] = std::move(plane);
        }

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

    void handle_status(iTFS::lite_device_t *device) {
        // Status handler: preserve the latest status and notify Python.
        if (direct_status_enabled_.load()) {
            StatusEvent event;
            event.device_index = device->idx;
            event.ip = ip_to_string(device->ip);
            event.port = device->port;
            event.status = device->status_v3;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
                latest_status_[event.device_index] = event;
            }
            invoke_event(1, device, nullptr);
            return;
        }

        StatusEvent event;
        event.device_index = device->idx;
        event.ip = ip_to_string(device->ip);
        event.port = device->port;
        event.status = device->status_v3;

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

    void handle_info(iTFS::lite_device_t *device) {
        // Info handler: preserve the latest info and notify Python.
        if (direct_info_enabled_.load()) {
            InfoEvent event;
            event.device_index = device->idx;
            event.ip = ip_to_string(device->ip);
            event.port = device->port;
            event.info = device->info_v3;
            {
                std::lock_guard<std::mutex> lk(queue_mutex_);
                update_device_snapshot(device);
                latest_info_[event.device_index] = event;
            }
            invoke_event(2, device, nullptr);
            return;
        }

        InfoEvent event;
        event.device_index = device->idx;
        event.ip = ip_to_string(device->ip);
        event.port = device->port;
        event.info = device->info_v3;

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

    static void image_callback(iTFS::lite_device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lite != nullptr) {
            g_active_lite->handle_image(device);
        }
    }

    static void status_callback(iTFS::lite_device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lite != nullptr) {
            g_active_lite->handle_status(device);
        }
    }

    static void info_callback(iTFS::lite_device_t *device) {
        std::lock_guard<std::mutex> lk(g_active_mutex);
        if (g_active_lite != nullptr) {
            g_active_lite->handle_info(device);
        }
    }

  private:
    static void validate_event_handler(const py::object &handler, const char *name) {
        if (!handler.is_none() && !py::isinstance<py::function>(handler)) {
            throw std::invalid_argument(std::string(name) + " must be callable or None");
        }
    }

    std::shared_ptr<std::vector<float>> get_reconstruction_map(const iTFS::lite_device_t *device) {
        auto it = reconstruction_maps_.find(device->idx);
        if (it != reconstruction_maps_.end() || !device->reconstruction_map.valid) {
            return it == reconstruction_maps_.end() ? nullptr : it->second;
        }

        auto map = std::make_shared<std::vector<float>>(static_cast<size_t>(kRows) * kCols * 3);
        for (int r = 0; r < kRows; r++) {
            for (int c = 0; c < kCols; c++) {
                size_t idx = static_cast<size_t>(r * kCols + c) * 3;
                (*map)[idx + 0] = device->reconstruction_map.dir[0][r][c];
                (*map)[idx + 1] = device->reconstruction_map.dir[1][r][c];
                (*map)[idx + 2] = device->reconstruction_map.dir[2][r][c];
            }
        }
        reconstruction_maps_[device->idx] = map;
        return map;
    }

    void invoke_event(int event_type,
                      iTFS::lite_device_t *device,
                      std::shared_ptr<std::vector<float>> reconstruction_map) {
        py::gil_scoped_acquire gil;
        py::object callback;
        {
            std::lock_guard<std::mutex> lk(event_mutex_);
            callback = event_type == 0 ? on_data_ : (event_type == 1 ? on_status_ : on_info_);
        }
        if (!callback) {
            return;
        }

        auto view = std::make_shared<LiteDevice>(device, std::move(reconstruction_map));
        try {
            callback(view);
        } catch (const py::error_already_set &e) {
            PyErr_WriteUnraisable(e.value().ptr());
        }
        view->invalidate();
    }

    void update_device_snapshot(const iTFS::lite_device_t *device) {
        auto &snapshot = devices_[device->idx];
        snapshot.index = device->idx;
        snapshot.ip = ip_to_string(device->ip);
        snapshot.port = device->port;
        snapshot.status = device->status_v3;
        snapshot.info = device->info_v3;
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

    std::unique_ptr<iTFS::LITE> lite_;
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
    std::unordered_map<int, std::shared_ptr<std::vector<float>>> reconstruction_maps_;
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
    d["LITE_MAX_ROW"] = iTFS::lite_max_row;
    d["LITE_MAX_COL"] = iTFS::lite_max_col;
    d["DEPTH_OFF"] = iTFS::packet::info_v3_data_output_depth_off;
    d["DEPTH_MM_16"] = iTFS::packet::info_v3_data_output_depth_mm_16bit;
    d["DEPTH_RAW_Q16"] = iTFS::packet::info_v3_data_output_depth_raw_q16;
    d["DEPTH_LIN_8"] = iTFS::packet::info_v3_data_output_depth_lin_8bit;
    d["DEPTH_LOG_8"] = iTFS::packet::info_v3_data_output_depth_log_8bit;
    d["XYZ_MM_16"] = iTFS::packet::info_v3_data_output_xyz_mm_16bit;
    d["XYZ_RAW_Q15_Q16"] = iTFS::packet::info_v3_data_output_xyz_raw_q15_q16;
    d["XYZ_LIN_8"] = iTFS::packet::info_v3_data_output_xyz_lin_8bit;
    d["AMPLITUDE_OFF"] = iTFS::packet::info_v3_data_output_amplitude_off;
    d["AMPLITUDE_RAW_16"] = iTFS::packet::info_v3_data_output_amplitude_raw_16bit;
    d["AMPLITUDE_LIN_8"] = iTFS::packet::info_v3_data_output_amplitude_lin_8bit;
    d["AMPLITUDE_LOG_8"] = iTFS::packet::info_v3_data_output_amplitude_log_8bit;
    d["INTENSITY_OFF"] = iTFS::packet::info_v3_data_output_intensity_off;
    d["INTENSITY_RAW_16"] = iTFS::packet::info_v3_data_output_intensity_raw_16bit;
    d["INTENSITY_LIN_8"] = iTFS::packet::info_v3_data_output_intensity_lin_8bit;
    d["INTENSITY_LOG_8"] = iTFS::packet::info_v3_data_output_intensity_log_8bit;
    d["CONFIDENCE_OFF"] = iTFS::packet::info_v3_data_output_confidence_off;
    d["CONFIDENCE_RAW_16"] = iTFS::packet::info_v3_data_output_confidence_raw_16bit;
    d["CONFIDENCE_MASK_1"] = iTFS::packet::info_v3_data_output_confidence_mask_1bit;
    return d;
}

} // namespace

PYBIND11_MODULE(_itfs_lite, m) {
    if (iTFS::lite_version() != 0) {
        throw std::runtime_error("iTFS-LITE library and packet header versions do not match");
    }

    m.doc() = "Native Python bindings for the iTFS-LITE C++ SDK";

    // Python-owned image copy overwritten by each received data event.
    py::class_<LiteImgCpy, std::shared_ptr<LiteImgCpy>>(m, "LiteImgCpy")
        .def(py::init<>())
        .def_property_readonly("idx", [](const LiteImgCpy &f) { return f.device_index; })
        .def_property_readonly("ip", [](const LiteImgCpy &f) { return f.ip; })
        .def_property_readonly("port", [](const LiteImgCpy &f) { return f.port; })
        .def_property_readonly("mode", [](const LiteImgCpy &f) { return f.image.mode; })
        .def_property_readonly("frame", [](const LiteImgCpy &f) { return f.image.frame; })
        .def_property_readonly("frame_status", [](const LiteImgCpy &f) { return f.image.frame_status; })
        .def_property_readonly("capture_row", [](const LiteImgCpy &f) { return f.capture_row; })
        .def_property_readonly("data_output", [](const LiteImgCpy &f) { return f.data_output; })
        .def_property_readonly("depth_on", [](const LiteImgCpy &f) { return f.image.depth_on; })
        .def_property_readonly("amplitude_on", [](const LiteImgCpy &f) { return f.image.amplitude_on; })
        .def_property_readonly("intensity_on", [](const LiteImgCpy &f) { return f.image.intensity_on; })
        .def_property_readonly("confidence_on", [](const LiteImgCpy &f) { return f.image.confidence_on; })
        .def_property_readonly("xyz_on", [](const LiteImgCpy &f) { return f.image.xyz_on; })
        .def_property_readonly("reconstruction_map_valid", [](const LiteImgCpy &f) {
            return static_cast<bool>(f.reconstruction_map);
        })
        .def_property_readonly("data_classes", [](const LiteImgCpy &f) {
            py::list out;
            for (int i = 0; i < f.image.data_class_count && i < iTFS::lite_img_class_count; i++) {
                out.append(class_name(f.image.data_class[i]));
            }
            return out;
        })
        .def_property_readonly("image", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_depth, self);
        })
        .def_property_readonly("depth", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_depth, self);
        })
        .def_property_readonly("amplitude", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_amplitude, self);
        })
        .def_property_readonly("intensity", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_intensity, self);
        })
        .def_property_readonly("confidence", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_confidence, self);
        })
        .def_property_readonly("point_x", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_point_x, self);
        })
        .def_property_readonly("point_y", [](py::object self) {
            return copied_image_to_numpy(self.cast<LiteImgCpy &>(), iTFS::lite_img_point_y, self);
        })
        .def_property_readonly("depth_display", [](const LiteImgCpy &f) {
            return copied_display_to_numpy(f, iTFS::lite_img_depth);
        })
        .def_property_readonly("amplitude_display", [](const LiteImgCpy &f) {
            return copied_display_to_numpy(f, iTFS::lite_img_amplitude);
        })
        .def_property_readonly("intensity_display", [](const LiteImgCpy &f) {
            return copied_display_to_numpy(f, iTFS::lite_img_intensity);
        })
        .def_property_readonly("confidence_display", [](const LiteImgCpy &f) {
            return copied_display_to_numpy(f, iTFS::lite_img_confidence);
        })
        .def_property_readonly("confidence_mask1", [](const LiteImgCpy &f) {
            uint16_t mode =
                (f.data_output & iTFS::packet::info_v3_data_output_confidence_mask) >>
                iTFS::packet::info_v3_data_output_confidence_pos;
            return mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;
        })
        .def_property_readonly("reconstruction_map", [](py::object self) -> py::object {
            auto &copy = self.cast<LiteImgCpy &>();
            if (!copy.reconstruction_map) {
                return py::none();
            }
            return py::array(
                py::dtype::of<float>(),
                {copy.capture_row, kCols, 3},
                {static_cast<py::ssize_t>(kCols * 3 * sizeof(float)),
                 static_cast<py::ssize_t>(3 * sizeof(float)),
                 static_cast<py::ssize_t>(sizeof(float))},
                copy.reconstruction_map->data(),
                self);
        })
        .def("point_cloud", &copied_point_cloud_to_numpy);

    // Live device view passed only while a direct event callback is running.
    py::class_<LiteDevice, std::shared_ptr<LiteDevice>>(m, "LiteDevice")
        .def_property_readonly("idx", &LiteDevice::device_index)
        .def_property_readonly("ip", &LiteDevice::ip)
        .def_property_readonly("port", &LiteDevice::port)
        .def_property_readonly("status", &LiteDevice::status)
        .def_property_readonly("info", &LiteDevice::info)
        .def("copy_image", &LiteDevice::copy_image, py::arg("target"));

    // Native receive runtime and its public Python API.
    py::class_<LITE>(m, "LITE")
        .def(py::init<size_t, py::object, py::object, uint16_t>(),
             py::arg("queue_size") = 2,
             py::arg("broadcast_ip") = py::none(),
             py::arg("listening_ip") = py::none(),
             py::arg("listening_port") = iTFS::user_data_port)
        .def("close", &LITE::close)
        .def("__enter__", [](LITE &self) -> LITE & { return self; })
        .def("__exit__", [](LITE &self, py::object, py::object, py::object) { self.close(); })
        .def_property_readonly("ready", &LITE::ready)
        .def_property_readonly("device_count", &LITE::device_count)
        .def("devices", &LITE::devices)
        .def("set_event_handlers", &LITE::set_event_handlers,
             py::arg("on_data") = py::none(),
             py::arg("on_status") = py::none(),
             py::arg("on_info") = py::none())
        .def("read_status", &LITE::read_status, py::arg("timeout") = 1.0, py::arg("idx") = py::none())
        .def("read_info", &LITE::read_info, py::arg("timeout") = 1.0, py::arg("idx") = py::none())
        .def("latest_status", &LITE::latest_status, py::arg("idx") = 0)
        .def("latest_info", &LITE::latest_info, py::arg("idx") = 0);

    m.def("make_capture_mode", &iTFS::make_lite_capture_mode,
          py::arg("freq"), py::arg("edge_filter"), py::arg("exposure"), py::arg("tof"));

    m.attr("max_device") = iTFS::max_device;
    m.attr("constants") = constants_dict();
}
