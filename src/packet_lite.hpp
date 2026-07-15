/**
 * @file packet_lite.hpp
 * @brief packet definitions for iTFS-LITE
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

#include "packet.hpp"

namespace iTFS {
namespace packet {
constexpr uint8_t ilidar_lite_pack_ver[3] = {2, 0, 0};

constexpr uint16_t img_data_v3_id = 0x0003; // LITE V1.2.0+
constexpr uint16_t img_data_v3_len = 1284;  // LITE V1.2.0+

constexpr uint16_t status_v3_id = 0x0013; // LITE V1.2.0+
constexpr uint16_t status_v3_len = 44;    // LITE V1.3.0+

constexpr uint16_t intrnisic_id = 0x0022; // LITE V1.2.0+
constexpr uint16_t intrnisic_len = 122;   // LITE V1.2.0+

constexpr uint16_t info_v3_id = 0x0023; // LITE V1.2.0+
constexpr uint16_t info_v3_len = 152;   // LITE V1.2.0+

typedef struct {
    uint8_t id;         // Image class id, see img_data_v3_class_*
    uint8_t row;        // Row index inside the image class
    uint8_t mode;       // Capture mode
    uint8_t frame;      // Frame number, repeats 0 ~ 255
    uint16_t data[640]; // 2 rows x 320 columns
} img_data_packet_v3_t;

typedef struct {
    uint8_t capture_mode;              // Mode
    uint8_t capture_frame;             // Frame number, repeats 0 ~ 255
    uint16_t sensor_sn;                // Serial number
    uint64_t sensor_time_th;           // Sensor time in [ms]
    uint16_t sensor_time_tl;           // Sensor time in [us]
    uint16_t sensor_frame_status;      // Sensor status flag for depth image
    int16_t sensor_temp_rx;            // Sensor RX temperature in [C] x100
    int16_t sensor_temp_tx;            // Sensor TX temperature in [C] x100
    int16_t sensor_temp_core;          // Sensor core temperature in [C] x100
    int16_t sensor_usb_level;          // Sensor usb voltage
    int16_t sensor_power_level[2];     // Sensor internal power voltage
    uint16_t sensor_cpu_usage;         // Sensor CPU usage in [%] x100
    uint16_t sensor_ram_usage;         // Sensor RAM usage in [%] x100
    uint32_t sensor_warning;           // Sensor warning flag
    uint16_t sensor_exposure;          // Effective exposure time in [us]
    uint16_t sensor_frame_drop_count;  // Frame drop/mismatch count
    uint16_t sensor_udp_rx_drop_count; // Network RX drop count
    uint16_t sensor_udp_tx_drop_count; // Network TX drop count
} status_v3_t;

typedef struct {              // [RW]
    uint16_t sensor_sn;       // [R-] Serial number
    uint8_t sensor_hw_id[64]; // [R-] HW ID
    uint8_t sensor_fw_ver[3]; // [R-] Firmware version
    char sensor_fw_date[12];  // [R-] Firmware date
    char sensor_fw_time[9];   // [R-] Firmware time
    uint8_t sensor_bl_ver[3]; // [R-] Bootloader version
    uint8_t sensor_model_id;  // [R-] Sensor model identifier
    uint8_t sensor_boot_ctrl; // [R-] Boot control/status

    uint8_t capture_mode;        // [RW] Firmware-specific capture mode
    uint8_t capture_row;         // [R-] Capture row count
    uint16_t capture_shutter[3]; // [RW] Capture shutter integration time in (us)
    uint16_t capture_limit;      // [RW] Capture limit
    uint32_t capture_period_ns;  // [RW] Capture period in (ns)

    uint16_t data_output;      // [RW] [3:0] depth/xyz, [5:4] amplitude, [7:6] intensity, [9:8] confidence
    uint8_t data_sensor_ip[4]; // [RW] Sensor IP
    uint8_t data_dest_ip[4];   // [RW] Destination IP
    uint8_t data_subnet[4];    // [RW] Subnet Mask
    uint8_t data_gateway[4];   // [RW] Gateway
    uint16_t data_port;        // [RW] Data output port number
    uint8_t data_mac_addr[6];  // [RW] MAC address of the sensor
    uint8_t data_dhcp_ctrl;    // [RW] 1 = DHCP, 0 = static

    uint8_t sync;                  // [RW] Sync flag
    uint16_t sync_param;           // [RW] Sync parameter
    uint32_t sync_trig_delay_us;   // [RW] Trigger delay in (us)
    uint16_t sync_output_delay_us; // [RW] Output delay in (us)

    uint8_t arb;          // [RW] Arbitration flag
    uint32_t arb_timeout; // [RW] Arbitration timeout
    uint8_t lock;         // [R-] Configuration locker
} info_v3_t;

typedef struct { // [R-]
    uint16_t sensor_sn;
    uint32_t lens_cx, lens_cy;
    uint32_t lens_fx, lens_fy;
    int32_t lens_kc[6];
    uint16_t fw_lens_cx, fw_lens_cy;
    uint16_t fw_lens_fx, fw_lens_fy;
    uint16_t fw_lens_kc_lsb[6];
    uint16_t fw_lens_kc_msb[6];
} intrinsic_t;

constexpr int info_v3_data_output_depth_pos = 0;
constexpr uint16_t info_v3_data_output_depth_mask = (0x0F << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_depth_off = (0x00 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_depth_mm_16bit = (0x01 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_depth_raw_q16 = (0x02 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_depth_lin_8bit = (0x03 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_depth_log_8bit = (0x04 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_xyz_mm_16bit = (0x05 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_xyz_raw_q15_q16 = (0x06 << info_v3_data_output_depth_pos);
constexpr uint16_t info_v3_data_output_xyz_lin_8bit = (0x07 << info_v3_data_output_depth_pos);

constexpr uint16_t info_v3_data_output_stream_mask = 0x03;
constexpr uint16_t info_v3_data_output_stream_off = 0x00;
constexpr uint16_t info_v3_data_output_stream_raw_16bit = 0x01;
constexpr uint16_t info_v3_data_output_stream_lin_8bit = 0x02;
constexpr uint16_t info_v3_data_output_stream_log_8bit = 0x03;

constexpr int info_v3_data_output_amplitude_pos = 4;
constexpr uint16_t info_v3_data_output_amplitude_mask = (info_v3_data_output_stream_mask << info_v3_data_output_amplitude_pos);
constexpr uint16_t info_v3_data_output_amplitude_off = (info_v3_data_output_stream_off << info_v3_data_output_amplitude_pos);
constexpr uint16_t info_v3_data_output_amplitude_raw_16bit = (info_v3_data_output_stream_raw_16bit << info_v3_data_output_amplitude_pos);
constexpr uint16_t info_v3_data_output_amplitude_lin_8bit = (info_v3_data_output_stream_lin_8bit << info_v3_data_output_amplitude_pos);
constexpr uint16_t info_v3_data_output_amplitude_log_8bit = (info_v3_data_output_stream_log_8bit << info_v3_data_output_amplitude_pos);

constexpr int info_v3_data_output_intensity_pos = 6;
constexpr uint16_t info_v3_data_output_intensity_mask = (info_v3_data_output_stream_mask << info_v3_data_output_intensity_pos);
constexpr uint16_t info_v3_data_output_intensity_off = (info_v3_data_output_stream_off << info_v3_data_output_intensity_pos);
constexpr uint16_t info_v3_data_output_intensity_raw_16bit = (info_v3_data_output_stream_raw_16bit << info_v3_data_output_intensity_pos);
constexpr uint16_t info_v3_data_output_intensity_lin_8bit = (info_v3_data_output_stream_lin_8bit << info_v3_data_output_intensity_pos);
constexpr uint16_t info_v3_data_output_intensity_log_8bit = (info_v3_data_output_stream_log_8bit << info_v3_data_output_intensity_pos);

constexpr int info_v3_data_output_confidence_pos = 8;
constexpr uint16_t info_v3_data_output_confidence_mask = (info_v3_data_output_stream_mask << info_v3_data_output_confidence_pos);
constexpr uint16_t info_v3_data_output_confidence_off = (info_v3_data_output_stream_off << info_v3_data_output_confidence_pos);
constexpr uint16_t info_v3_data_output_confidence_raw_16bit = (info_v3_data_output_stream_raw_16bit << info_v3_data_output_confidence_pos);
constexpr uint16_t info_v3_data_output_confidence_mask_1bit_mode = 0x02;
constexpr uint16_t info_v3_data_output_confidence_mask_1bit = (info_v3_data_output_confidence_mask_1bit_mode << info_v3_data_output_confidence_pos);

static void decode_status_v3(uint8_t *src, status_v3_t *dst) {
    dst->capture_mode = src[0];
    dst->capture_frame = src[1];
    dst->sensor_sn = (src[3] << 8) | src[2];
    uint64_t th = src[11];
    for (int _i = 0; _i < 7; _i++) {
        th <<= 8;
        th |= src[10 - _i];
    }
    dst->sensor_time_th = th;
    dst->sensor_time_tl = (src[13] << 8) | src[12];
    dst->sensor_frame_status = (src[15] << 8) | src[14];
    dst->sensor_temp_rx = (src[17] << 8) | src[16];
    dst->sensor_temp_tx = (src[19] << 8) | src[18];
    dst->sensor_temp_core = (src[21] << 8) | src[20];
    dst->sensor_usb_level = (src[23] << 8) | src[22];
    dst->sensor_power_level[0] = (src[25] << 8) | src[24];
    dst->sensor_power_level[1] = (src[27] << 8) | src[26];
    dst->sensor_cpu_usage = (src[29] << 8) | src[28];
    dst->sensor_ram_usage = (src[31] << 8) | src[30];
    dst->sensor_warning = (src[35] << 24) | (src[34] << 16) | (src[33] << 8) | src[32];
    dst->sensor_exposure = (src[37] << 8) | src[36];
    dst->sensor_frame_drop_count = (src[39] << 8) | src[38];
    dst->sensor_udp_rx_drop_count = (src[41] << 8) | src[40];
    dst->sensor_udp_tx_drop_count = (src[43] << 8) | src[42];
}

static void encode_info_v3(info_v3_t *src, uint8_t *dst) {
    dst[0] = (src->sensor_sn >> 0) & 0xFF;
    dst[1] = (src->sensor_sn >> 8) & 0xFF;
    for (int _i = 0; _i < 64; _i++) {
        dst[2 + _i] = src->sensor_hw_id[_i];
    }
    dst[66] = src->sensor_fw_ver[0];
    dst[67] = src->sensor_fw_ver[1];
    dst[68] = src->sensor_fw_ver[2];
    for (int _i = 0; _i < 12; _i++) {
        dst[69 + _i] = src->sensor_fw_date[_i];
    }
    for (int _i = 0; _i < 9; _i++) {
        dst[81 + _i] = src->sensor_fw_time[_i];
    }
    dst[90] = src->sensor_bl_ver[0];
    dst[91] = src->sensor_bl_ver[1];
    dst[92] = src->sensor_bl_ver[2];
    dst[93] = src->sensor_model_id;
    dst[94] = src->sensor_boot_ctrl;

    dst[95] = src->capture_mode;
    dst[96] = src->capture_row;
    dst[97] = (src->capture_shutter[0] >> 0) & 0xFF;
    dst[98] = (src->capture_shutter[0] >> 8) & 0xFF;
    dst[99] = (src->capture_shutter[1] >> 0) & 0xFF;
    dst[100] = (src->capture_shutter[1] >> 8) & 0xFF;
    dst[101] = (src->capture_shutter[2] >> 0) & 0xFF;
    dst[102] = (src->capture_shutter[2] >> 8) & 0xFF;
    dst[103] = (src->capture_limit >> 0) & 0xFF;
    dst[104] = (src->capture_limit >> 8) & 0xFF;
    dst[105] = (src->capture_period_ns >> 0) & 0xFF;
    dst[106] = (src->capture_period_ns >> 8) & 0xFF;
    dst[107] = (src->capture_period_ns >> 16) & 0xFF;
    dst[108] = (src->capture_period_ns >> 24) & 0xFF;

    dst[109] = (src->data_output >> 0) & 0xFF;
    dst[110] = (src->data_output >> 8) & 0xFF;
    for (int _i = 0; _i < 4; _i++) {
        dst[111 + _i] = src->data_sensor_ip[_i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst[115 + _i] = src->data_dest_ip[_i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst[119 + _i] = src->data_subnet[_i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst[123 + _i] = src->data_gateway[_i];
    }
    dst[127] = (src->data_port >> 0) & 0xFF;
    dst[128] = (src->data_port >> 8) & 0xFF;
    for (int _i = 0; _i < 6; _i++) {
        dst[129 + _i] = src->data_mac_addr[_i];
    }
    dst[135] = src->data_dhcp_ctrl ? 1 : 0;

    dst[136] = src->sync;
    dst[137] = (src->sync_param >> 0) & 0xFF;
    dst[138] = (src->sync_param >> 8) & 0xFF;
    dst[139] = (src->sync_trig_delay_us >> 0) & 0xFF;
    dst[140] = (src->sync_trig_delay_us >> 8) & 0xFF;
    dst[141] = (src->sync_trig_delay_us >> 16) & 0xFF;
    dst[142] = (src->sync_trig_delay_us >> 24) & 0xFF;
    dst[143] = (src->sync_output_delay_us >> 0) & 0xFF;
    dst[144] = (src->sync_output_delay_us >> 8) & 0xFF;
    dst[145] = src->arb;
    dst[146] = (src->arb_timeout >> 0) & 0xFF;
    dst[147] = (src->arb_timeout >> 8) & 0xFF;
    dst[148] = (src->arb_timeout >> 16) & 0xFF;
    dst[149] = (src->arb_timeout >> 24) & 0xFF;
    dst[150] = src->lock;
    dst[151] = 0; // Padding byte for even payload length
}

static void decode_info_v3(uint8_t *src, info_v3_t *dst) {
    dst->sensor_sn = (src[1] << 8) | src[0];
    for (int _i = 0; _i < 64; _i++) {
        dst->sensor_hw_id[_i] = src[2 + _i];
    }
    dst->sensor_fw_ver[0] = src[66];
    dst->sensor_fw_ver[1] = src[67];
    dst->sensor_fw_ver[2] = src[68];
    for (int _i = 0; _i < 12; _i++) {
        dst->sensor_fw_date[_i] = src[69 + _i];
    }
    for (int _i = 0; _i < 9; _i++) {
        dst->sensor_fw_time[_i] = src[81 + _i];
    }
    dst->sensor_bl_ver[0] = src[90];
    dst->sensor_bl_ver[1] = src[91];
    dst->sensor_bl_ver[2] = src[92];
    dst->sensor_model_id = src[93];
    dst->sensor_boot_ctrl = src[94];

    dst->capture_mode = src[95];
    dst->capture_row = src[96];
    dst->capture_shutter[0] = (src[98] << 8) | src[97];
    dst->capture_shutter[1] = (src[100] << 8) | src[99];
    dst->capture_shutter[2] = (src[102] << 8) | src[101];
    dst->capture_limit = (src[104] << 8) | src[103];
    dst->capture_period_ns = (src[108] << 24) | (src[107] << 16) | (src[106] << 8) | src[105];

    dst->data_output = (src[110] << 8) | src[109];
    for (int _i = 0; _i < 4; _i++) {
        dst->data_sensor_ip[_i] = src[111 + _i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst->data_dest_ip[_i] = src[115 + _i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst->data_subnet[_i] = src[119 + _i];
    }
    for (int _i = 0; _i < 4; _i++) {
        dst->data_gateway[_i] = src[123 + _i];
    }
    dst->data_port = (src[128] << 8) | src[127];
    for (int _i = 0; _i < 6; _i++) {
        dst->data_mac_addr[_i] = src[129 + _i];
    }
    dst->data_dhcp_ctrl = src[135] ? 1 : 0;

    dst->sync = src[136];
    dst->sync_param = (src[138] << 8) | src[137];
    dst->sync_trig_delay_us = (src[142] << 24) | (src[141] << 16) | (src[140] << 8) | src[139];
    dst->sync_output_delay_us = (src[144] << 8) | src[143];
    dst->arb = src[145];
    dst->arb_timeout = (src[149] << 24) | (src[148] << 16) | (src[147] << 8) | src[146];
    dst->lock = src[150];
}

static void decode_intrinsic(uint8_t *src, intrinsic_t *dst) {
    uint16_t sensor_sn = (src[1] << 8) | src[0];

    uint32_t lens_cx = (src[5] << 24) | (src[4] << 16) | (src[3] << 8) | src[2];
    uint32_t lens_cy = (src[9] << 24) | (src[8] << 16) | (src[7] << 8) | src[6];

    uint32_t lens_fx = (src[13] << 24) | (src[12] << 16) | (src[11] << 8) | src[10];
    uint32_t lens_fy = (src[17] << 24) | (src[16] << 16) | (src[15] << 8) | src[14];

    uint32_t lens_kc1 = (src[21] << 24) | (src[20] << 16) | (src[19] << 8) | src[18];
    uint32_t lens_kc2 = (src[25] << 24) | (src[24] << 16) | (src[23] << 8) | src[22];
    uint32_t lens_kc3 = (src[29] << 24) | (src[28] << 16) | (src[27] << 8) | src[26];
    uint32_t lens_kc4 = (src[33] << 24) | (src[32] << 16) | (src[31] << 8) | src[30];
    uint32_t lens_kc5 = (src[37] << 24) | (src[36] << 16) | (src[35] << 8) | src[34];
    uint32_t lens_kc6 = (src[41] << 24) | (src[40] << 16) | (src[39] << 8) | src[38];

    uint16_t fw_lens_cx = (src[43] << 8) | src[42];
    uint16_t fw_lens_cy = (src[45] << 8) | src[44];

    uint16_t fw_lens_fx = (src[47] << 8) | src[46];
    uint16_t fw_lens_fy = (src[49] << 8) | src[48];

    uint16_t fw_lens_kc1_lsb = (src[51] << 8) | src[50];
    uint16_t fw_lens_kc1_msb = (src[53] << 8) | src[52];
    uint16_t fw_lens_kc2_lsb = (src[55] << 8) | src[54];
    uint16_t fw_lens_kc2_msb = (src[57] << 8) | src[56];
    uint16_t fw_lens_kc3_lsb = (src[59] << 8) | src[58];
    uint16_t fw_lens_kc3_msb = (src[61] << 8) | src[60];
    uint16_t fw_lens_kc4_lsb = (src[63] << 8) | src[62];
    uint16_t fw_lens_kc4_msb = (src[65] << 8) | src[64];
    uint16_t fw_lens_kc5_lsb = (src[67] << 8) | src[66];
    uint16_t fw_lens_kc5_msb = (src[69] << 8) | src[68];
    uint16_t fw_lens_kc6_lsb = (src[71] << 8) | src[70];
    uint16_t fw_lens_kc6_msb = (src[73] << 8) | src[72];

    dst->sensor_sn = sensor_sn;

    dst->lens_cx = lens_cx;
    dst->lens_cy = lens_cy;

    dst->lens_fx = lens_fx;
    dst->lens_fy = lens_fy;

    dst->lens_kc[0] = lens_kc1;
    dst->lens_kc[1] = lens_kc2;
    dst->lens_kc[2] = lens_kc3;
    dst->lens_kc[3] = lens_kc4;
    dst->lens_kc[4] = lens_kc5;
    dst->lens_kc[5] = lens_kc6;

    dst->fw_lens_cx = fw_lens_cx;
    dst->fw_lens_cy = fw_lens_cy;

    dst->fw_lens_fx = fw_lens_fx;
    dst->fw_lens_fy = fw_lens_fy;

    dst->fw_lens_kc_lsb[0] = fw_lens_kc1_lsb;
    dst->fw_lens_kc_lsb[1] = fw_lens_kc2_lsb;
    dst->fw_lens_kc_lsb[2] = fw_lens_kc3_lsb;
    dst->fw_lens_kc_lsb[3] = fw_lens_kc4_lsb;
    dst->fw_lens_kc_lsb[4] = fw_lens_kc5_lsb;
    dst->fw_lens_kc_lsb[5] = fw_lens_kc6_lsb;

    dst->fw_lens_kc_msb[0] = fw_lens_kc1_msb;
    dst->fw_lens_kc_msb[1] = fw_lens_kc2_msb;
    dst->fw_lens_kc_msb[2] = fw_lens_kc3_msb;
    dst->fw_lens_kc_msb[3] = fw_lens_kc4_msb;
    dst->fw_lens_kc_msb[4] = fw_lens_kc5_msb;
    dst->fw_lens_kc_msb[5] = fw_lens_kc6_msb;
}

/*
    // About iLidar sensor time
    uint64_t    sensor_time_th;    // [ms] Sensor time in ms
    uint16_t    sensor_time_tl;    // [us] Sensor time for sub-ms

    // So, we can get the sensor time in s
    double        sensor_time = ((double)sensor_time_th + (((double)sensor_time_tl) * 0.001)) * 0.001;

    // When we need to use the precise time, we can use time in us with integer
    uint64_t    sensor_time_us = (uint64_t)(1000 * sensor_time_th) + (uint64_t)sensor_time_tl;
*/

static double get_sensor_time(status_v3_t *status) {
    return (((double)status->sensor_time_th + ((double)status->sensor_time_tl) * 0.001) * 0.001);
}

static uint64_t get_sensor_time_in_us(status_v3_t *status) {
    return ((uint64_t)(1000 * status->sensor_time_th) + (uint64_t)status->sensor_time_tl);
}
} // namespace packet
} // namespace iTFS
