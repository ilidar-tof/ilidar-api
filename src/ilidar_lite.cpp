/**
 * @file ilidar_lite.cpp
 * @brief iTFS-LITE v3 receiver
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

#include "ilidar_lite.hpp"

#include <stdlib.h>

#if defined(_WIN32) || defined(_WIN64)
#include <iphlpapi.h>
#include <ws2tcpip.h>
#pragma comment(lib, "iphlpapi.lib")
#else
#include <ifaddrs.h>
#include <net/if.h>
#endif

namespace iTFS {
LITE::LITE(
    lite_callback_handler img_data_handler_func,
    lite_callback_handler status_packet_handler_func,
    lite_callback_handler info_packet_handler_func,
    uint8_t *brodcast_ip,
    uint8_t *listening_ip,
    uint16_t listening_port) {

    this->img_data_handler_func = img_data_handler_func;
    this->status_packet_handler_func = status_packet_handler_func;
    this->info_packet_handler_func = info_packet_handler_func;

    memset((void *)this->device, 0x00, sizeof(this->device));

    this->is_ready = false;
    this->exit = false;
    this->is_read_ready = false;
    this->is_send_ready = false;

    this->device_cnt = 0;
    this->broadcast_ip_cnt = 0;

    if (brodcast_ip == NULL) {
        // Default mode: discover every active IPv4 adapter and send broadcast per subnet.
        this->Update_broadcast_ip_list();
        printf("[MESSAGE] iTFS::LITE broadcast IP list has been updated. Count: %d\n", this->broadcast_ip_cnt);
    } else {
        // Manual mode: use only the caller-provided broadcast IP.
        this->Set_broadcast_ip(brodcast_ip);
        printf("[MESSAGE] iTFS::LITE unique broadcast IP has been set: %d.%d.%d.%d\n",
               this->broadcast_ip[0][0], this->broadcast_ip[0][1], this->broadcast_ip[0][2], this->broadcast_ip[0][3]);
    }

    if (listening_ip == NULL) {
        this->listening_ip[0] = 0;
        this->listening_ip[1] = 0;
        this->listening_ip[2] = 0;
        this->listening_ip[3] = 0;
    } else {
        this->listening_ip[0] = listening_ip[0];
        this->listening_ip[1] = listening_ip[1];
        this->listening_ip[2] = listening_ip[2];
        this->listening_ip[3] = listening_ip[3];
        printf("[MESSAGE] iTFS::LITE unique listening IP has been set: %d.%d.%d.%d\n",
               this->listening_ip[0], this->listening_ip[1], this->listening_ip[2], this->listening_ip[3]);
    }

    this->listening_port = listening_port;

    if (this->img_data_handler_func == NULL ||
        this->status_packet_handler_func == NULL ||
        this->info_packet_handler_func == NULL) {
        printf("[ ERROR ] iTFS::LITE callback functions must be defined.\n");
        this->is_ready = false;
        return;
    }

    this->read_thread = std::thread([=] { this->Read_run(); });
    this->send_thread = std::thread([=] { this->Send_run(); });

    auto pri_time = std::chrono::system_clock::now();
    while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        if (this->is_read_ready == true &&
            this->is_send_ready == true) {
            break;
        }

        auto cur_time = std::chrono::system_clock::now();
        auto try_time = std::chrono::duration_cast<std::chrono::milliseconds>(cur_time - pri_time);
        if (try_time.count() > 500) {
            this->is_ready = false;
            return;
        }
    }

    this->is_ready = true;
}

LITE::~LITE() {
    this->is_ready = false;
    this->Try_exit();
    this->Join();
}

void LITE::Read_run() {
    SOCKET_TYPE sockfd;
    int read_length;
    struct sockaddr_in addr, addr_client;
    SOCKET_LEN length = sizeof(addr_client);
    long recv_addr_bin;
    uint8_t *recv_addr = (uint8_t *)&recv_addr_bin;
    uint16_t recv_port;
    int recv_device;

#if defined(_WIN32) || defined(_WIN64)
    WSADATA wsa_data;
    if (WSAStartup(MAKEWORD(2, 2), &wsa_data) != NO_ERROR) {
        printf("[ERROR] iTFS::LITE WSA Loading Failed!\n");
        return;
    }
#endif // WSA

    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (SOCKET_INVALID(sockfd)) {
        printf("[ERROR] iTFS::LITE Receiver Socket Opening Failed\n");
        return;
    }

    memset((void *)&addr, 0x00, sizeof(addr));
    addr.sin_family = AF_INET;
    if (this->listening_ip[0] == 0 &&
        this->listening_ip[1] == 0 &&
        this->listening_ip[2] == 0 &&
        this->listening_ip[3] == 0) {
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
    } else {
        char addr_str[32];
        sprintf(addr_str, "%d.%d.%d.%d", this->listening_ip[0], this->listening_ip[1], this->listening_ip[2], this->listening_ip[3]);
        addr.sin_addr.s_addr = inet_addr(addr_str);
    }
    addr.sin_port = htons(this->listening_port);

    int enable = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&enable, sizeof(enable)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Receiver Socket Setsockopt Failed\n");
        closesocket(sockfd);
        return;
    }

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000;
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(timeout)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Receiver Socket Setsockopt Failed\n");
        closesocket(sockfd);
        return;
    }

    if (bind(sockfd, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Receiver Socket Bind Failed\n");
        closesocket(sockfd);
        return;
    }

    this->is_read_ready = true;

    unsigned char buffer[2000] = {
        0,
    };
    int timeout_cnt = 0;

    while (this->exit == false) {
        if ((read_length = recvfrom(sockfd, (char *)buffer, 2000, 0, (struct sockaddr *)&addr_client, &length)) < 0) {
            if (++timeout_cnt > 10) {
                printf("[MESSAGE] iTFS::LITE recvfrom: No data has been received in the last 5 seconds\n");
                timeout_cnt = 0;
            }
        } else {
            timeout_cnt = 0;

            recv_addr_bin = addr_client.sin_addr.s_addr;
            recv_port = htons(addr_client.sin_port);

            uint16_t message_id = (buffer[3] << 8) | (buffer[2]);
            uint16_t payload_len = (buffer[5] << 8) | (buffer[4]);
            bool valid = (buffer[0] == packet::stx0) && (buffer[1] == packet::stx1) &&
                         (buffer[read_length - 2] == packet::etx0) && (buffer[read_length - 1] == packet::etx1) &&
                         (payload_len == (read_length - packet::overheader_len));

            recv_device = this->Get_device_num(recv_addr, recv_port);

            if (recv_device < 0) {
                // LITE devices are registered by status_v3. Image packets before this point are ignored.
                if (valid && (message_id == packet::status_v3_id) && (payload_len == packet::status_v3_len)) {
                    recv_device = this->Set_device_num(recv_addr, recv_port);

                    if (recv_device >= 0) {
                        printf("[MESSAGE] iTFS::LITE A new device is registered. D#%d: %3d.%3d.%3d.%3d:%5d\n",
                               recv_device, recv_addr[0], recv_addr[1], recv_addr[2], recv_addr[3], recv_port);

                        packet::decode_status_v3((uint8_t *)&buffer[6], &(this->device[recv_device].status_v3));
                        this->device[recv_device].data.capture_row = lite_max_row;
                    }
                }
                continue;
            }

            if (valid && (message_id == packet::img_data_v3_id) && (payload_len == packet::img_data_v3_len)) {
                uint8_t id = buffer[6];
                uint8_t row = buffer[7];
                uint8_t mode = buffer[8];
                uint8_t frame = buffer[9];
                uint16_t data_output = this->device[recv_device].info_v3.data_output;
                uint16_t depth_mode = data_output & packet::info_v3_data_output_depth_mask;
                uint16_t amplitude_mode = (data_output & packet::info_v3_data_output_amplitude_mask) >> packet::info_v3_data_output_amplitude_pos;
                uint16_t intensity_mode = (data_output & packet::info_v3_data_output_intensity_mask) >> packet::info_v3_data_output_intensity_pos;
                uint16_t confidence_mode = (data_output & packet::info_v3_data_output_confidence_mask) >> packet::info_v3_data_output_confidence_pos;

                this->device[recv_device].data.mode = mode;
                this->device[recv_device].data.frame = frame;

                if ((id < lite_img_class_count) && (this->device[recv_device].data.img_offset[id] >= 0)) {
                    bool img8_packet =
                        ((id == lite_img_depth) && (depth_mode == packet::info_v3_data_output_depth_lin_8bit ||
                                                    depth_mode == packet::info_v3_data_output_depth_log_8bit ||
                                                    depth_mode == packet::info_v3_data_output_xyz_lin_8bit)) ||
                        ((id == lite_img_point_x || id == lite_img_point_y) && depth_mode == packet::info_v3_data_output_xyz_lin_8bit) ||
                        ((id == lite_img_amplitude) && (amplitude_mode == packet::info_v3_data_output_stream_lin_8bit ||
                                                        amplitude_mode == packet::info_v3_data_output_stream_log_8bit)) ||
                        ((id == lite_img_intensity) && (intensity_mode == packet::info_v3_data_output_stream_lin_8bit ||
                                                        intensity_mode == packet::info_v3_data_output_stream_log_8bit));
                    bool mask1_packet =
                        (id == lite_img_confidence) &&
                        (confidence_mode == packet::info_v3_data_output_confidence_mask_1bit_mode);

                    if (mask1_packet) {
                        int image_row = row * 32;
                        int rows = (image_row + 32 <= lite_max_row) ? 32 : (lite_max_row - image_row);
                        uint8_t (*image_u8)[lite_max_col] = get_img_u8_ptr(&this->device[recv_device].data, id);
                        const uint8_t *src = &buffer[10];

                        for (int r = 0; r < rows; r++) {
                            const uint8_t *src_row = src + r * 40;
                            uint8_t *dst_row = &image_u8[image_row + r][0];

                            for (int b = 0; b < 40; b++) {
                                uint8_t packed = src_row[b];

                                for (int bit = 0; bit < 8; bit++) {
                                    dst_row[b * 8 + bit] = static_cast<uint8_t>((packed >> bit) & 0x01);
                                }
                            }

                            this->device[recv_device].data.row_frame[id][image_row + r] = this->device[recv_device].data.frame;
                        }
                    } else if (img8_packet) {
                        int image_row = row * 4;
                        uint8_t (*image_u8)[lite_max_col] = get_img_u8_ptr(&this->device[recv_device].data, id);

                        memcpy((void *)&image_u8[image_row][0],
                               (const void *)&buffer[10],
                               sizeof(uint8_t) * 4 * lite_max_col);

                        this->device[recv_device].data.row_frame[id][image_row + 0] = this->device[recv_device].data.frame;
                        this->device[recv_device].data.row_frame[id][image_row + 1] = this->device[recv_device].data.frame;
                        this->device[recv_device].data.row_frame[id][image_row + 2] = this->device[recv_device].data.frame;
                        this->device[recv_device].data.row_frame[id][image_row + 3] = this->device[recv_device].data.frame;
                    } else {
                        int image_row = row * 2;
                        uint16_t (*image_u16)[lite_max_col] = get_img_u16_ptr(&this->device[recv_device].data, id);

                        memcpy((void *)&image_u16[image_row][0],
                               (const void *)&buffer[10],
                               sizeof(uint16_t) * 2 * lite_max_col);

                        this->device[recv_device].data.row_frame[id][image_row + 0] = this->device[recv_device].data.frame;
                        this->device[recv_device].data.row_frame[id][image_row + 1] = this->device[recv_device].data.frame;
                    }
                }
                continue;
            } else if (valid && (message_id == packet::status_v3_id) && (payload_len == packet::status_v3_len)) {
                // Firmware sends status_v3 after image packets; use it as the frame-complete signal.
                packet::decode_status_v3((uint8_t *)&buffer[6], &(this->device[recv_device].status_v3));

                this->device[recv_device].data.mode = this->device[recv_device].status_v3.capture_mode;
                this->device[recv_device].data.frame = this->device[recv_device].status_v3.capture_frame;
                this->device[recv_device].data.frame_status = 0;

                for (int _slot = 0; _slot < this->device[recv_device].data.data_class_count; _slot++) {
                    uint8_t class_id = this->device[recv_device].data.data_class[_slot];

                    // Missing-row check: all rows for this class must match frame number.
                    int sum_row_frames = 0;
                    for (int _i = 0; _i < this->device[recv_device].data.capture_row; _i++) {
                        sum_row_frames += this->device[recv_device].data.row_frame[class_id][_i];
                    }

                    if (sum_row_frames != (this->device[recv_device].data.frame * this->device[recv_device].data.capture_row)) {
                        this->device[recv_device].data.frame_status = status_missing_rows;
                        printf("[WARNING] iTFS::LITE D#%d frame %u has missing rows.\n", recv_device, this->device[recv_device].data.frame);
                        break;
                    }
                }

                if (this->device[recv_device].status_v3.sensor_sn != this->device[recv_device].info_v3.sensor_sn) {
                    iTFS::packet::cmd_t read_info;
                    read_info.cmd_id = iTFS::packet::cmd_read_info;
                    read_info.cmd_msg = 0;
                    this->Send_cmd(recv_device, &read_info);
                    printf("[MESSAGE] iTFS::LITE cmd_read_info packet was sent.\n");
                }

                this->img_data_handler_func(&this->device[recv_device]);
                this->status_packet_handler_func(&this->device[recv_device]);
                continue;
            } else if (valid && (message_id == packet::info_v3_id) && (payload_len == packet::info_v3_len)) {
                packet::decode_info_v3((uint8_t *)&buffer[6], &(this->device[recv_device].info_v3));

                uint16_t data_output = this->device[recv_device].info_v3.data_output;
                int capture_row = this->device[recv_device].info_v3.capture_row;
                if (capture_row <= 0 || capture_row > lite_max_row) {
                    capture_row = lite_max_row;
                }

                uint16_t depth_mode = data_output & packet::info_v3_data_output_depth_mask;
                uint16_t amplitude_mode = (data_output & packet::info_v3_data_output_amplitude_mask) >> packet::info_v3_data_output_amplitude_pos;
                uint16_t intensity_mode = (data_output & packet::info_v3_data_output_intensity_mask) >> packet::info_v3_data_output_intensity_pos;
                uint16_t confidence_mode = (data_output & packet::info_v3_data_output_confidence_mask) >> packet::info_v3_data_output_confidence_pos;

                this->device[recv_device].data.capture_row = capture_row;
                this->device[recv_device].data.depth_on =
                    (depth_mode == packet::info_v3_data_output_depth_mm_16bit ||
                     depth_mode == packet::info_v3_data_output_depth_raw_q16 ||
                     depth_mode == packet::info_v3_data_output_depth_lin_8bit ||
                     depth_mode == packet::info_v3_data_output_depth_log_8bit);
                this->device[recv_device].data.amplitude_on =
                    (amplitude_mode == packet::info_v3_data_output_stream_raw_16bit ||
                     amplitude_mode == packet::info_v3_data_output_stream_lin_8bit ||
                     amplitude_mode == packet::info_v3_data_output_stream_log_8bit);
                this->device[recv_device].data.intensity_on =
                    (intensity_mode == packet::info_v3_data_output_stream_raw_16bit ||
                     intensity_mode == packet::info_v3_data_output_stream_lin_8bit ||
                     intensity_mode == packet::info_v3_data_output_stream_log_8bit);
                this->device[recv_device].data.confidence_on =
                    (confidence_mode == packet::info_v3_data_output_stream_raw_16bit ||
                     confidence_mode == packet::info_v3_data_output_confidence_mask_1bit_mode);
                this->device[recv_device].data.xyz_on =
                    (depth_mode == packet::info_v3_data_output_xyz_mm_16bit ||
                     depth_mode == packet::info_v3_data_output_xyz_raw_q15_q16 ||
                     depth_mode == packet::info_v3_data_output_xyz_lin_8bit);

                this->device[recv_device].data.data_class_count = 0;
                for (int _i = 0; _i < lite_img_class_count; _i++) {
                    this->device[recv_device].data.img_offset[_i] = (-1);
                }

                if (this->device[recv_device].data.depth_on || this->device[recv_device].data.xyz_on) {
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_depth;
                    this->device[recv_device].data.img_offset[lite_img_depth] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                }
                if (this->device[recv_device].data.amplitude_on) {
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_amplitude;
                    this->device[recv_device].data.img_offset[lite_img_amplitude] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                }
                if (this->device[recv_device].data.intensity_on) {
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_intensity;
                    this->device[recv_device].data.img_offset[lite_img_intensity] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                }
                if (this->device[recv_device].data.confidence_on) {
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_confidence;
                    this->device[recv_device].data.img_offset[lite_img_confidence] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                }
                if (this->device[recv_device].data.xyz_on) {
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_point_x;
                    this->device[recv_device].data.img_offset[lite_img_point_x] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                    this->device[recv_device].data.data_class[this->device[recv_device].data.data_class_count++] = lite_img_point_y;
                    this->device[recv_device].data.img_offset[lite_img_point_y] = (this->device[recv_device].data.data_class_count - 1) * lite_max_row;
                }

                this->info_packet_handler_func(&this->device[recv_device]);
                continue;
            } else if (valid && (message_id == packet::intrnisic_id) && (payload_len == packet::intrnisic_len)) {
                // Decode message
                packet::decode_intrinsic((uint8_t *)&buffer[6], &(this->device[recv_device].intrinsic));

                // Generate reconstruction map
                this->Set_reconstruction_map(recv_device);
                continue;
            }
        }
    }

    this->is_read_ready = false;
    closesocket(sockfd);

#if defined(_WIN32) || defined(_WIN64)
    WSACleanup();
#endif // WSA
}

void LITE::Send_run() {
    // Wait for Read_run() thread initialization
    while (this->is_read_ready == false) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

#if defined(_WIN32) || defined(_WIN64)
    // Skip to call WSA startup - already called in Read_run()
#endif // WSA

    this->send_sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (SOCKET_INVALID(this->send_sockfd)) {
        printf("[ERROR] iTFS::LITE Sender Socket Opening Failed\n");
        return;
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(user_config_port);

    int enable = 1;

    if (setsockopt(this->send_sockfd, SOL_SOCKET, SO_REUSEADDR, (char *)&enable, sizeof(enable)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Sender Socket Setsockopt Failed\n");
        closesocket(this->send_sockfd);
        return;
    }

    if (setsockopt(this->send_sockfd, SOL_SOCKET, SO_BROADCAST, (char *)&enable, sizeof(enable)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Sender Socket Setsockopt Failed\n");
        closesocket(this->send_sockfd);
        return;
    }

    if (bind(this->send_sockfd, (struct sockaddr *)&addr, sizeof(addr)) == SOCKET_ERROR_RET) {
        printf("[ERROR] iTFS::LITE Socket Bind Failed\n");
        closesocket(this->send_sockfd);
        return;
    }

    this->is_send_ready = true;

    while (this->exit == false) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    this->is_send_ready = false;
    closesocket(this->send_sockfd);

#if defined(_WIN32) || defined(_WIN64)
    WSACleanup();
#endif // WSA
}

int LITE::Get_device_num(uint8_t ip[4], uint16_t port) {
    for (int _i = 0; _i < this->device_cnt; _i++) {
        if ((ip[0] == this->device[_i].ip[0]) &&
            (ip[1] == this->device[_i].ip[1]) &&
            (ip[2] == this->device[_i].ip[2]) &&
            (ip[3] == this->device[_i].ip[3]) &&
            (port == this->device[_i].port)) {
            return _i;
        }
    }
    return (-1);
}

int LITE::Set_device_num(uint8_t ip[4], uint16_t port) {
    if (Get_device_num(ip, port) != (-1)) {
        return (-1);
    }

    if (this->device_cnt >= max_device) {
        return (-2);
    }

    int _i = this->device_cnt;

    this->device[_i].ip[0] = ip[0];
    this->device[_i].ip[1] = ip[1];
    this->device[_i].ip[2] = ip[2];
    this->device[_i].ip[3] = ip[3];
    this->device[_i].port = port;

    this->device[_i].idx = _i;
    this->device[_i].info_v3.sensor_sn = 0xFFFF;
    this->device[_i].data.capture_row = lite_max_row;
    this->device[_i].data.data_class_count = 0;
    for (int _j = 0; _j < lite_img_class_count; _j++) {
        this->device[_i].data.img_offset[_j] = (-1);
    }

    this->device_cnt++;

    return _i;
}

int LITE::Send_cmd(int device_idx, packet::cmd_t *cmd) {
    if (device_idx >= this->device_cnt) {
        return (0);
    }

    return this->Send_cmd(this->device[device_idx].ip, cmd);
}

int LITE::Send_cmd(uint8_t *device_ip, packet::cmd_t *cmd) {
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));

    uint32_t ip = (device_ip[3] << 24) |
                  (device_ip[2] << 16) |
                  (device_ip[1] << 8) |
                  (device_ip[0] << 0);

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ip;
    addr.sin_port = htons(lidar_config_port);

    uint8_t buffer[16];
    buffer[0] = packet::stx0;
    buffer[1] = packet::stx1;
    buffer[2] = (packet::cmd_id >> 0) & 0xFF;
    buffer[3] = (packet::cmd_id >> 8) & 0xFF;
    buffer[4] = (packet::cmd_len >> 0) & 0xFF;
    buffer[5] = (packet::cmd_len >> 8) & 0xFF;
    buffer[packet::header_len + packet::cmd_len] = packet::etx0;
    buffer[packet::header_len + packet::cmd_len + 1] = packet::etx1;
    packet::encode_cmd(cmd, &buffer[packet::header_len]);

    this->send_mutex.lock();
    int result = sendto(this->send_sockfd, (const char *)buffer, (packet::overheader_len + packet::cmd_len), 0, (struct sockaddr *)&addr, sizeof(addr));
    this->send_mutex.unlock();

    return result;
}

int LITE::Send_cmd_to_all(packet::cmd_t *cmd) {
    uint8_t buffer[16];
    buffer[0] = packet::stx0;
    buffer[1] = packet::stx1;
    buffer[2] = (packet::cmd_id >> 0) & 0xFF;
    buffer[3] = (packet::cmd_id >> 8) & 0xFF;
    buffer[4] = (packet::cmd_len >> 0) & 0xFF;
    buffer[5] = (packet::cmd_len >> 8) & 0xFF;
    buffer[packet::header_len + packet::cmd_len] = packet::etx0;
    buffer[packet::header_len + packet::cmd_len + 1] = packet::etx1;
    packet::encode_cmd(cmd, &buffer[packet::header_len]);

    this->send_mutex.lock();

    int result = 0;
    for (int _i = 0; _i < this->broadcast_ip_cnt; _i++) {
        struct sockaddr_in addr;
        memset(&addr, 0, sizeof(addr));

        uint32_t ip = (this->broadcast_ip[_i][3] << 24) |
                      (this->broadcast_ip[_i][2] << 16) |
                      (this->broadcast_ip[_i][1] << 8) |
                      (this->broadcast_ip[_i][0] << 0);

        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = ip;
        addr.sin_port = htons(lidar_config_port);

        int sent = sendto(this->send_sockfd, (const char *)buffer, (packet::overheader_len + packet::cmd_len), 0, (struct sockaddr *)&addr, sizeof(addr));
        if (sent > 0) {
            result += sent;
        } else if (result == 0) {
            result = sent;
        }
    }

    this->send_mutex.unlock();

    return result;
}

int LITE::Send_config(int device_idx, packet::info_v3_t *config) {
    if (device_idx >= this->device_cnt) {
        return (0);
    }

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));

    uint32_t ip = (this->device[device_idx].ip[3] << 24) |
                  (this->device[device_idx].ip[2] << 16) |
                  (this->device[device_idx].ip[1] << 8) |
                  (this->device[device_idx].ip[0] << 0);

    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = ip;
    addr.sin_port = htons(lidar_config_port);

    uint8_t buffer[256];
    buffer[0] = packet::stx0;
    buffer[1] = packet::stx1;
    buffer[2] = (packet::info_v3_id >> 0) & 0xFF;
    buffer[3] = (packet::info_v3_id >> 8) & 0xFF;
    buffer[4] = (packet::info_v3_len >> 0) & 0xFF;
    buffer[5] = (packet::info_v3_len >> 8) & 0xFF;
    buffer[packet::header_len + packet::info_v3_len] = packet::etx0;
    buffer[packet::header_len + packet::info_v3_len + 1] = packet::etx1;
    packet::encode_info_v3(config, &buffer[packet::header_len]);

    this->send_mutex.lock();
    int result = sendto(this->send_sockfd, (const char *)buffer, (packet::overheader_len + packet::info_v3_len), 0, (struct sockaddr *)&addr, sizeof(addr));
    this->send_mutex.unlock();

    return result;
}

void LITE::Set_broadcast_ip(uint8_t *ip) {
    this->broadcast_ip_cnt = 1;
    this->broadcast_ip[0][0] = ip[0];
    this->broadcast_ip[0][1] = ip[1];
    this->broadcast_ip[0][2] = ip[2];
    this->broadcast_ip[0][3] = ip[3];
}

int LITE::Set_reconstruction_map(int device_idx) {
    float cx = this->device[device_idx].intrinsic.lens_cx * 1e-5f;
    float cy = this->device[device_idx].intrinsic.lens_cy * 1e-5f;
    float fx = this->device[device_idx].intrinsic.lens_fx * 1e-5f;
    float fy = this->device[device_idx].intrinsic.lens_fy * 1e-5f;

    float k1 = this->device[device_idx].intrinsic.lens_kc[0] * 1e-8f;
    float k2 = this->device[device_idx].intrinsic.lens_kc[1] * 1e-8f;
    float p1 = this->device[device_idx].intrinsic.lens_kc[2] * 1e-8f;
    float p2 = this->device[device_idx].intrinsic.lens_kc[3] * 1e-8f;
    float k3 = this->device[device_idx].intrinsic.lens_kc[4] * 1e-8f;
    float k4 = this->device[device_idx].intrinsic.lens_kc[5] * 1e-8f;

    float map_cx = (iTFS::lite_max_col - 1) - cx;
    float map_cy = (iTFS::lite_max_row - 1) - cy;
    float map_fx = fx;
    float map_fy = fy;

    for (int _v = 0; _v < iTFS::lite_max_row; _v++) {
        for (int _u = 0; _u < iTFS::lite_max_col; _u++) {
            float x = (_u - map_cx) / map_fx;
            float y = (_v - map_cy) / map_fy;

            float r2 = x * x + y * y;
            float r4 = r2 * r2;
            float r6 = r4 * r2;

            float radial = (1.0f + k1 * r2 + k2 * r4 + k3 * r6 + k4 * r6 * r2);

            float x_dist = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x);
            float y_dist = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y;

            float rx = x_dist;
            float ry = y_dist;
            float rz = 1.0f; // We reconstruct depth-z image

            this->device[device_idx].reconstruction_map.dir[0][_v][_u] = rx;
            this->device[device_idx].reconstruction_map.dir[1][_v][_u] = ry;
            this->device[device_idx].reconstruction_map.dir[2][_v][_u] = rz;
        }
    }

    this->device[device_idx].reconstruction_map.valid = true;

    return 0;
}

void LITE::Update_broadcast_ip_list() {
    this->broadcast_ip_cnt = 0;

#if defined(_WIN32) || defined(_WIN64)
    ULONG flags = GAA_FLAG_SKIP_ANYCAST | GAA_FLAG_SKIP_MULTICAST | GAA_FLAG_SKIP_DNS_SERVER;
    ULONG family = AF_INET;
    ULONG buffer_size = 15000;
    IP_ADAPTER_ADDRESSES *addresses = (IP_ADAPTER_ADDRESSES *)malloc(buffer_size);

    if (addresses == NULL) {
        return;
    }

    ULONG ret = GetAdaptersAddresses(family, flags, NULL, addresses, &buffer_size);
    if (ret == ERROR_BUFFER_OVERFLOW) {
        free(addresses);
        addresses = (IP_ADAPTER_ADDRESSES *)malloc(buffer_size);
        if (addresses == NULL) {
            return;
        }
        ret = GetAdaptersAddresses(family, flags, NULL, addresses, &buffer_size);
    }

    if (ret == NO_ERROR) {
        for (IP_ADAPTER_ADDRESSES *adapter = addresses; adapter != NULL; adapter = adapter->Next) {
            if (adapter->OperStatus != IfOperStatusUp) {
                continue;
            }
            if (adapter->IfType == IF_TYPE_SOFTWARE_LOOPBACK) {
                continue;
            }

            for (IP_ADAPTER_UNICAST_ADDRESS *unicast = adapter->FirstUnicastAddress; unicast != NULL; unicast = unicast->Next) {
                if (unicast->Address.lpSockaddr == NULL || unicast->Address.lpSockaddr->sa_family != AF_INET) {
                    continue;
                }
                if (unicast->OnLinkPrefixLength == 0 || unicast->OnLinkPrefixLength > 32) {
                    continue;
                }

                sockaddr_in *addr = (sockaddr_in *)unicast->Address.lpSockaddr;
                uint32_t ip = ntohl(addr->sin_addr.s_addr);
                uint32_t mask = (unicast->OnLinkPrefixLength == 32) ? 0xFFFFFFFFU : (0xFFFFFFFFU << (32 - unicast->OnLinkPrefixLength));
                uint32_t broadcast = ip | (~mask);

                if (broadcast == ip) {
                    continue;
                }

                bool duplicated = false;
                for (int _i = 0; _i < this->broadcast_ip_cnt; _i++) {
                    uint32_t exist = (this->broadcast_ip[_i][0] << 24) | (this->broadcast_ip[_i][1] << 16) | (this->broadcast_ip[_i][2] << 8) | this->broadcast_ip[_i][3];
                    if (exist == broadcast) {
                        duplicated = true;
                        break;
                    }
                }
                if (duplicated || this->broadcast_ip_cnt >= lite_max_broadcast_ip) {
                    continue;
                }

                int idx = this->broadcast_ip_cnt++;
                this->broadcast_ip[idx][0] = (broadcast >> 24) & 0xFF;
                this->broadcast_ip[idx][1] = (broadcast >> 16) & 0xFF;
                this->broadcast_ip[idx][2] = (broadcast >> 8) & 0xFF;
                this->broadcast_ip[idx][3] = (broadcast >> 0) & 0xFF;
            }
        }
    }

    free(addresses);
#else
    struct ifaddrs *ifaddr = NULL;
    if (getifaddrs(&ifaddr) != 0) {
        return;
    }

    for (struct ifaddrs *ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == NULL || ifa->ifa_netmask == NULL) {
            continue;
        }
        if (ifa->ifa_addr->sa_family != AF_INET) {
            continue;
        }
        if ((ifa->ifa_flags & IFF_UP) == 0 || (ifa->ifa_flags & IFF_LOOPBACK) != 0) {
            continue;
        }

        sockaddr_in *addr = (sockaddr_in *)ifa->ifa_addr;
        sockaddr_in *mask_addr = (sockaddr_in *)ifa->ifa_netmask;
        uint32_t ip = ntohl(addr->sin_addr.s_addr);
        uint32_t mask = ntohl(mask_addr->sin_addr.s_addr);
        uint32_t broadcast = ip | (~mask);

        if (broadcast == ip) {
            continue;
        }

        bool duplicated = false;
        for (int _i = 0; _i < this->broadcast_ip_cnt; _i++) {
            uint32_t exist = (this->broadcast_ip[_i][0] << 24) | (this->broadcast_ip[_i][1] << 16) | (this->broadcast_ip[_i][2] << 8) | this->broadcast_ip[_i][3];
            if (exist == broadcast) {
                duplicated = true;
                break;
            }
        }
        if (duplicated || this->broadcast_ip_cnt >= lite_max_broadcast_ip) {
            continue;
        }

        int idx = this->broadcast_ip_cnt++;
        this->broadcast_ip[idx][0] = (broadcast >> 24) & 0xFF;
        this->broadcast_ip[idx][1] = (broadcast >> 16) & 0xFF;
        this->broadcast_ip[idx][2] = (broadcast >> 8) & 0xFF;
        this->broadcast_ip[idx][3] = (broadcast >> 0) & 0xFF;
    }

    freeifaddrs(ifaddr);
#endif

    if (this->broadcast_ip_cnt == 0) {
        uint8_t default_broadcast[4] = {255, 255, 255, 255};
        this->Set_broadcast_ip(default_broadcast);
    }
}
} // namespace iTFS
