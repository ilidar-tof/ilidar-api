/**
 * @file ilidar_lite.hpp
 * @brief iTFS-LITE v3 receiver header
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

#pragma once

#include "amp_int_log8_lut_lite.hpp"
#include "depth_log8_lut_lite.hpp"
#include "ilidar.hpp"
#include "packet_lite.hpp"

namespace iTFS {
constexpr uint8_t ilidar_lite_lib_ver[3] = {2, 0, 0};

static int lite_version(void) {
    printf("[MESSAGE] iTFS::LITE | ilidar_lite.cpp V%d.%d.%d\n",
           ilidar_lite_lib_ver[0], ilidar_lite_lib_ver[1], ilidar_lite_lib_ver[2]);
    printf("[MESSAGE] iTFS::LITE | packet_lite.hpp V%d.%d.%d\n",
           packet::ilidar_lite_pack_ver[0], packet::ilidar_lite_pack_ver[1], packet::ilidar_lite_pack_ver[2]);

    if (!((ilidar_lite_lib_ver[0] == packet::ilidar_lite_pack_ver[0]) &&
          (ilidar_lite_lib_ver[1] == packet::ilidar_lite_pack_ver[1]) &&
          (ilidar_lite_lib_ver[2] == packet::ilidar_lite_pack_ver[2]))) {
        printf("[ ERROR ] iTFS::LITE | The versions of ilidar_lite.cpp and packet_lite.hpp do not match.\n");
        return -1;
    }

    return 0;
}

constexpr int lite_max_col = 320;
constexpr int lite_max_row = 240;

constexpr int lite_img_class_count = 6;
constexpr int lite_max_broadcast_ip = 16;

constexpr int lite_img_depth = 0;
constexpr int lite_img_amplitude = 1;
constexpr int lite_img_intensity = 2;
constexpr int lite_img_confidence = 3;
constexpr int lite_img_point_x = 4;
constexpr int lite_img_point_y = 5;

/*
 * info_v3.capture_mode bit layout
 * --------------------------------
 * [7:6] frequency
 *   0: dual frequency, 1: F1 single, 2: F2 single, 3: reserved
 * [5] dust filter
 *   0: off, 1: on
 * [4] edge filter
 *   0: off, 1: on
 * [3:2] exposure mode
 *   0: auto, 1: fixed, 2: HDR LV2, 3: HDR LV3
 * [1:0] ToF processing mode
 *   0: gray, 1: normal, 2: vertical binning, 3: horizontal+vertical binning
 */
constexpr int lite_capture_mode_freq_pos = 6;
constexpr uint8_t lite_capture_mode_freq_mask = 0xC0;
constexpr uint8_t lite_capture_mode_freq_dual = 0;
constexpr uint8_t lite_capture_mode_freq_f1_single = 1;
constexpr uint8_t lite_capture_mode_freq_f2_single = 2;
constexpr uint8_t lite_capture_mode_freq_reserved = 3;

constexpr int lite_capture_mode_dust_filter_pos = 5;
constexpr uint8_t lite_capture_mode_dust_filter_mask = 0x20;
constexpr uint8_t lite_capture_mode_dust_filter_off = 0;
constexpr uint8_t lite_capture_mode_dust_filter_on = 1;

constexpr int lite_capture_mode_edge_filter_pos = 4;
constexpr uint8_t lite_capture_mode_edge_filter_mask = 0x10;
constexpr uint8_t lite_capture_mode_edge_filter_off = 0;
constexpr uint8_t lite_capture_mode_edge_filter_on = 1;

constexpr int lite_capture_mode_exposure_pos = 2;
constexpr uint8_t lite_capture_mode_exposure_mask = 0x0C;
constexpr uint8_t lite_capture_mode_exposure_auto = 0;
constexpr uint8_t lite_capture_mode_exposure_fixed = 1;
constexpr uint8_t lite_capture_mode_exposure_hdr_lv2 = 2;
constexpr uint8_t lite_capture_mode_exposure_hdr_lv3 = 3;

constexpr int lite_capture_mode_tof_pos = 0;
constexpr uint8_t lite_capture_mode_tof_mask = 0x03;
constexpr uint8_t lite_capture_mode_tof_gray = 0;
constexpr uint8_t lite_capture_mode_tof_normal = 1;
constexpr uint8_t lite_capture_mode_tof_v_bin = 2;
constexpr uint8_t lite_capture_mode_tof_hv_bin = 3;

constexpr uint8_t make_lite_capture_mode(uint8_t freq,
                                         uint8_t edge_filter,
                                         uint8_t exposure,
                                         uint8_t tof) {
    return (uint8_t)((((freq) << lite_capture_mode_freq_pos) & lite_capture_mode_freq_mask) |
                     (((edge_filter) << lite_capture_mode_edge_filter_pos) & lite_capture_mode_edge_filter_mask) |
                     (((exposure) << lite_capture_mode_exposure_pos) & lite_capture_mode_exposure_mask) |
                     (((tof) << lite_capture_mode_tof_pos) & lite_capture_mode_tof_mask));
}

/*
 * info_v3.data_output quick reference
 * -----------------------------------
 * [3:0] depth/XYZ:
 *   OFF, DEPTH_MM_16, DEPTH_RAW_Q16, DEPTH_LIN_8, DEPTH_LOG_8,
 *   XYZ_MM_16, XYZ_RAW_Q15_Q16, XYZ_LIN_8
 * [5:4] amplitude:
 *   OFF, RAW_16, LIN_8, LOG_8
 * [7:6] intensity:
 *   OFF, RAW_16, LIN_8, LOG_8
 * [9:8] confidence:
 *   OFF, RAW_16, MASK_1
 *
 * Use iTFS::packet::info_v3_data_output_* constants from packet_lite.hpp
 * when composing data_output.
 */

typedef struct {
    float dir[3][lite_max_row][lite_max_col];
    bool valid;
} lite_vec3d_t;

typedef union {
    uint16_t u16[lite_max_row][lite_max_col];
    int16_t s16[lite_max_row][lite_max_col];
    uint8_t u8[lite_max_row][lite_max_col];
    int8_t s8[lite_max_row][lite_max_col];
} lite_img_slot_t;

typedef struct {
    uint8_t mode;
    uint8_t frame;
    uint8_t data_class_count;
    uint8_t data_class[lite_img_class_count];
    int img_offset[lite_img_class_count];

    int frame_status;

    bool depth_on;
    bool amplitude_on;
    bool intensity_on;
    bool confidence_on;
    bool xyz_on;

    lite_img_slot_t data[lite_img_class_count];
} lite_img_cpy_t;

typedef struct {
    uint8_t mode;
    uint8_t frame;
    uint8_t row_frame[lite_img_class_count][lite_max_row];
    uint8_t data_class[lite_img_class_count];
    uint8_t data_class_count;
    int img_offset[lite_img_class_count];

    int capture_row;

    int frame_status;

    bool depth_on;
    bool amplitude_on;
    bool intensity_on;
    bool confidence_on;
    bool xyz_on;

    lite_img_slot_t data[lite_img_class_count];
} lite_img_t;

inline int copy_img_metadata(lite_img_cpy_t *dst, lite_img_t *src) {
    if (dst == NULL || src == NULL) {
        return (-1);
    }

    dst->mode = src->mode;
    dst->frame = src->frame;

    dst->data_class_count = src->data_class_count;

    dst->img_offset[0] = src->img_offset[0];
    dst->img_offset[1] = src->img_offset[1];
    dst->img_offset[2] = src->img_offset[2];
    dst->img_offset[3] = src->img_offset[3];
    dst->img_offset[4] = src->img_offset[4];
    dst->img_offset[5] = src->img_offset[5];

    dst->data_class[0] = src->data_class[0];
    dst->data_class[1] = src->data_class[1];
    dst->data_class[2] = src->data_class[2];
    dst->data_class[3] = src->data_class[3];
    dst->data_class[4] = src->data_class[4];
    dst->data_class[5] = src->data_class[5];

    dst->frame_status = src->frame_status;

    dst->depth_on = src->depth_on;
    dst->amplitude_on = src->amplitude_on;
    dst->intensity_on = src->intensity_on;
    dst->confidence_on = src->confidence_on;
    dst->xyz_on = src->xyz_on;

    return 0;
}

inline uint16_t (*get_img_u16_ptr(lite_img_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].u16;
}

inline uint16_t (*get_img_u16_ptr(lite_img_cpy_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].u16;
}

inline int16_t (*get_img_s16_ptr(lite_img_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].s16;
}

inline int16_t (*get_img_s16_ptr(lite_img_cpy_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].s16;
}

inline uint8_t (*get_img_u8_ptr(lite_img_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].u8;
}

inline uint8_t (*get_img_u8_ptr(lite_img_cpy_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].u8;
}

inline int8_t (*get_img_s8_ptr(lite_img_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].s8;
}

inline int8_t (*get_img_s8_ptr(lite_img_cpy_t *data, uint8_t img_class)) [lite_max_col] {
    if (img_class >= lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }
    return data->data[data->img_offset[img_class] / lite_max_row].s8;
}

typedef struct {
    int idx;

    uint8_t ip[4];
    uint16_t port;

    int recvCount;

    lite_img_t data;
    lite_vec3d_t reconstruction_map;

    packet::status_v3_t status_v3;
    packet::info_v3_t info_v3;
    packet::intrinsic_t intrinsic;
} lite_device_t;

typedef void (*lite_callback_handler)(lite_device_t *);

class LITE {
  public:
    using depth_log8_lut_lite = iTFS::depth_log8_lut_lite;
    using amp_int_log8_lut_lite = iTFS::amp_int_log8_lut_lite;

    LITE(
        lite_callback_handler img_data_handler_func,
        lite_callback_handler status_packet_handler_func,
        lite_callback_handler info_packet_handler_func,
        uint8_t *brodcast_ip = NULL,
        uint8_t *listening_ip = NULL,
        uint16_t listening_port = user_data_port);

    ~LITE();

    bool Ready() { return is_ready; }
    void Try_exit() { this->exit = true; }
    void Join() {
        if (this->read_thread.joinable()) {
            this->read_thread.join();
        }
        if (this->send_thread.joinable()) {
            this->send_thread.join();
        }
    }

    int Send_cmd(int device_idx, packet::cmd_t *cmd);
    int Send_cmd(uint8_t *device_ip, packet::cmd_t *cmd);
    int Send_cmd_to_all(packet::cmd_t *cmd);

    int Send_config(int device_idx, packet::info_v3_t *config);

    void Set_broadcast_ip(uint8_t *ip);

    int device_cnt;
    lite_device_t device[max_device];

  private:
    bool is_ready;
    bool exit;

    std::thread read_thread;
    std::thread send_thread;

    bool is_read_ready;
    bool is_send_ready;

    void Read_run();
    void Send_run();

    uint8_t listening_ip[4];
    uint16_t listening_port;

    std::mutex send_mutex;
    SOCKET_TYPE send_sockfd;

    int broadcast_ip_cnt;
    uint8_t broadcast_ip[lite_max_broadcast_ip][4];
    void Update_broadcast_ip_list();

    int Get_device_num(uint8_t ip[4], uint16_t port);
    int Set_device_num(uint8_t ip[4], uint16_t port);

    int Set_reconstruction_map(int device_idx);

    lite_callback_handler img_data_handler_func;
    lite_callback_handler status_packet_handler_func;
    lite_callback_handler info_packet_handler_func;

    void Copy_status_v3(lite_device_t *device);
};
} // namespace iTFS
