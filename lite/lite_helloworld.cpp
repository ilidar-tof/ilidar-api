/**
 * @file lite_helloworld.cpp
 * @brief iTFS-LITE helloworld example
 * @author Junwoo Son (json@hybo.co)
 * @date 2026-09-09
 * @version 2.0.1
 */

#include <chrono>
#include <condition_variable> // Data synchronization
#include <mutex>              // Data synchronization
#include <queue>              // Data synchronization
#include <stdio.h>
#include <thread>

#include "../src/ilidar_lite.hpp"

// One reusable image copy per SDK device. The receive callback overwrites it.
static iTFS::lite_img_cpy_t lite_img_data[iTFS::max_device];

// The queue carries device indexes only; active image slots stay in lite_img_data.
static std::condition_variable lidar_cv;
static std::mutex lidar_cv_mutex;
static std::queue<int> lidar_q;

/*
 * LITE image layout
 * -----------------
 * LITE can output several image classes in one frame:
 *   D = depth, A = amplitude, I = intensity, C = confidence, X/Y = point cloud axes.
 *
 * Only enabled classes are stored in data[], and they are packed in this fixed order:
 *   D -> A -> I -> C -> X -> Y
 *
 * Each enabled class occupies one slot:
 *   data[slot].u16[row][col] for 16-bit images
 *   data[slot].u8[row][col]  for 8-bit images
 *
 * data_class[slot] tells which class is stored in each packed slot.
 * img_offset[class_id] is a row-style offset; divide it by lite_max_row to get the data[] slot.
 * If a class is disabled, img_offset[class_id] is -1.
 * Do not access data[img_offset[class_id] / lite_max_row] unless the offset is >= 0.
 *
 * Example 1: D and A are enabled
 *   data_class[0] = img_data_v3_class_depth
 *   data_class[1] = img_data_v3_class_amplitude
 *   img_offset[img_data_v3_class_depth]     = 0 * lite_max_row
 *   img_offset[img_data_v3_class_amplitude] = 1 * lite_max_row
 *
 * Example 2: D and C are enabled
 *   data_class[0] = img_data_v3_class_depth
 *   data_class[1] = img_data_v3_class_confidence
 *   img_offset[img_data_v3_class_depth]     = 0 * lite_max_row
 *   img_offset[img_data_v3_class_confidence]= 1 * lite_max_row
 *   img_offset[img_data_v3_class_amplitude] = -1
 *   The amplitude image is disabled, so it cannot be accessed.
 *
 * Recommended access:
 *   uint16_t (*depth)[iTFS::lite_max_col] =
 *       iTFS::get_img_u16_ptr(&data, iTFS::lite_img_depth);
 *   if (depth != NULL) {
 *       uint16_t pixel = depth[row][col];
 *   }
 *
 * Direct offset access is also possible:
 *   int off = data.img_offset[iTFS::packet::img_data_v3_class_depth];
 *   if (off >= 0) {
 *       uint16_t pixel = data.data[off / iTFS::lite_max_row].u16[row][col];
 *   }
 *
 * Full-copy note:
 *   lite_img_t stores offsets, not pointers, so memcpy() can safely copy it.
 */

// Example callback declarations. Implementations are kept below main()
// so the user-facing example body appears first.
static void lidar_data_handler(iTFS::lite_device_t *device);
static void status_packet_handler(iTFS::lite_device_t *device);
static void info_packet_handler(iTFS::lite_device_t *device);
static void keyboard_input_run(iTFS::LITE *ilidar);

// Main example starts here
int main(int argc, char *argv[]) {
    if (iTFS::lite_version() != 0) {
        return -1;
    }

    // Create iTFS LITE class
    iTFS::LITE *lite;
    lite = new iTFS::LITE(
        lidar_data_handler,
        status_packet_handler,
        info_packet_handler);

    // Check the sensor driver is ready
    while (lite->Ready() != true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    printf("[MESSAGE] iTFS::LITE is ready.\n");

    // Create keyboard input thread
    std::thread keyboard_input_thread = std::thread([=] { keyboard_input_run(lite); });

    // Main loop starts here
    int recv_device_idx = 0;
    while (true) {
        // Wait for new data
        std::unique_lock<std::mutex> lk(lidar_cv_mutex);
        lidar_cv.wait(lk, [] { return !lidar_q.empty(); });
        recv_device_idx = lidar_q.front();
        lidar_q.pop();

        // Discard stale notifications when processing falls behind.
        if (!lidar_q.empty()) {
            /* The main loop is slower than data reception handler */
            printf("[WARNING] iTFS::LITE The main loop seems to be slower than the LITE data reception handler.\n");

            // Flush the queue
            while (!lidar_q.empty()) {
                recv_device_idx = lidar_q.front();
                lidar_q.pop();
            }
        }

        /*** USER PROCESSING STARTS HERE ***/
        printf("[MESSAGE] iTFS::LITE image   | D# %d  M %d  F# %3d\n",
               recv_device_idx,
               lite_img_data[recv_device_idx].mode,
               lite_img_data[recv_device_idx].frame);

        // Sleep for other thread
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        /*** USER PROCESSING ENDS HERE ***/
    }

    // Stop and delete iTFS LITE class
    delete lite;
    printf("[MESSAGE] iTFS::LITE has been deleted.\n");

    // Wait for keyboard input thread
    keyboard_input_thread.join();

    return 0;
}

// Status and info packet print helper functions
static const char *depth_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_depth_off:
        return "OFF";
    case iTFS::packet::info_v3_data_output_depth_mm_16bit:
        return "DEPTH_MM_16";
    case iTFS::packet::info_v3_data_output_depth_raw_q16:
        return "DEPTH_RAW_Q16";
    case iTFS::packet::info_v3_data_output_depth_lin_8bit:
        return "DEPTH_LIN_8";
    case iTFS::packet::info_v3_data_output_depth_log_8bit:
        return "DEPTH_LOG_8";
    case iTFS::packet::info_v3_data_output_xyz_mm_16bit:
        return "XYZ_MM_16";
    case iTFS::packet::info_v3_data_output_xyz_raw_q15_q16:
        return "XYZ_RAW_Q15_Q16";
    case iTFS::packet::info_v3_data_output_xyz_lin_8bit:
        return "XYZ_LIN_8";
    default:
        return "UNKNOWN";
    }
}

static const char *stream_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_stream_off:
        return "OFF";
    case iTFS::packet::info_v3_data_output_stream_raw_16bit:
        return "RAW_16";
    case iTFS::packet::info_v3_data_output_stream_lin_8bit:
        return "LIN_8";
    case iTFS::packet::info_v3_data_output_stream_log_8bit:
        return "LOG_8";
    default:
        return "UNKNOWN";
    }
}

static const char *confidence_output_name(uint16_t mode) {
    switch (mode) {
    case iTFS::packet::info_v3_data_output_confidence_off >> iTFS::packet::info_v3_data_output_confidence_pos:
        return "OFF";
    case iTFS::packet::info_v3_data_output_confidence_raw_16bit >> iTFS::packet::info_v3_data_output_confidence_pos:
        return "RAW_16";
    case iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode:
        return "MASK_1";
    default:
        return "UNKNOWN";
    }
}

static const char *capture_freq_name(uint8_t mode) {
    switch ((mode >> 6) & 0x03) {
    case 0:
        return "DUAL";
    case 1:
        return "F1_SINGLE";
    case 2:
        return "F2_SINGLE";
    default:
        return "RESERVED";
    }
}

static const char *capture_filter_name(uint8_t bit) {
    return bit ? "ON" : "OFF";
}

static const char *capture_exposure_name(uint8_t mode) {
    switch ((mode >> 2) & 0x03) {
    case 0:
        return "AUTO";
    case 1:
        return "FIXED";
    case 2:
        return "HDR_LV2";
    default:
        return "HDR_LV3";
    }
}

static const char *capture_tof_name(uint8_t mode) {
    switch (mode & 0x03) {
    case 0:
        return "GRAY";
    case 1:
        return "NORMAL";
    case 2:
        return "V_BIN";
    default:
        return "HV_BIN";
    }
}

// Image callback: copy active SDK-owned slots, then wake the main loop.
static void lidar_data_handler(iTFS::lite_device_t *device) {
    iTFS::lite_img_t *src = &device->data;
    iTFS::lite_img_cpy_t *dst = &lite_img_data[device->idx];

    // Call metadata copy helper function of the lite data
    iTFS::copy_img_metadata(dst, src);

    // Deep-copy the lite data
    memcpy((void *)dst->data,
           (const void *)src->data,
           sizeof(iTFS::lite_img_slot_t) * dst->data_class_count);

    // Notify the reception to the main thread
    int idx = device->idx;
    std::lock_guard<std::mutex> lk(lidar_cv_mutex);
    lidar_q.push(idx);
    lidar_cv.notify_one();
}

// Status callback: inspect status_v3 here; do not retain device.
static void status_packet_handler(iTFS::lite_device_t *device) {
    unsigned long long time_us = (unsigned long long)get_sensor_time_in_us(&device->status_v3);

    printf("[MESSAGE] iTFS::LITE status  D# %d  M %3d  F# %3d  t %llu.%03llu sec\n",
           device->idx,
           device->status_v3.capture_mode,
           device->status_v3.capture_frame,
           time_us / 1000000ULL,
           (time_us / 1000ULL) % 1000ULL);
    printf("                             TEMP %6.2f %6.2f %6.2f C  EXP %3u us\n",
           (float)device->status_v3.sensor_temp_rx * 0.01f,
           (float)device->status_v3.sensor_temp_tx * 0.01f,
           (float)device->status_v3.sensor_temp_core * 0.01f,
           device->status_v3.sensor_exposure);
    printf("                             VOLT  %5.2f  %5.2f  %5.2f V  USE %6.2f %6.2f %%\n",
           (float)device->status_v3.sensor_usb_level * 0.01f,
           (float)device->status_v3.sensor_power_level[0] * 0.01f,
           (float)device->status_v3.sensor_power_level[1] * 0.01f,
           (float)device->status_v3.sensor_cpu_usage * 0.01f,
           (float)device->status_v3.sensor_ram_usage * 0.01f);
    printf("                             DROP %3u %3u %3u             WARN 0x%08X\n",
           device->status_v3.sensor_frame_drop_count,
           device->status_v3.sensor_udp_rx_drop_count,
           device->status_v3.sensor_udp_tx_drop_count,
           (unsigned int)device->status_v3.sensor_warning);
}

// Info callback: inspect info_v3 here; do not retain device.
static void info_packet_handler(iTFS::lite_device_t *device) {
    uint16_t data_output = device->info_v3.data_output;
    uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
    uint16_t amplitude_mode = (data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >> iTFS::packet::info_v3_data_output_amplitude_pos;
    uint16_t intensity_mode = (data_output & iTFS::packet::info_v3_data_output_intensity_mask) >> iTFS::packet::info_v3_data_output_intensity_pos;
    uint16_t confidence_mode = (data_output & iTFS::packet::info_v3_data_output_confidence_mask) >> iTFS::packet::info_v3_data_output_confidence_pos;
    uint8_t capture_mode = device->info_v3.capture_mode;

    printf("[MESSAGE] iTFS::LITE info_v3 packet was received.\n");
    printf("[MESSAGE] iTFS::LITE info_v3 %.*s was detected.\n",
           24,
           (const char *)device->info_v3.sensor_hw_id);
    printf("                             SN# %u  D# %d  LOCK %s(0x%02X)\n",
           device->info_v3.sensor_sn,
           device->idx,
           capture_filter_name(device->info_v3.lock),
           device->info_v3.lock);
    printf("[MESSAGE] iTFS::LITE info_v3 CAPTURE_MODE: %u(0x%02X)\n",
           capture_mode, capture_mode);
    printf("                             FREQ: %s(%u)\n",
           capture_freq_name(capture_mode), (capture_mode >> 6) & 0x03);
    printf("                             EDGE FILTER: %s(%u)\n",
           capture_filter_name((capture_mode >> 4) & 0x01), (capture_mode >> 4) & 0x01);
    printf("                             DUST FILTER: %s(%u)\n",
           capture_filter_name((capture_mode >> 5) & 0x01), (capture_mode >> 5) & 0x01);
    printf("                             EXPOSURE: %s(%u)\n",
           capture_exposure_name(capture_mode), (capture_mode >> 2) & 0x03);
    printf("                             TOF: %s(%u)\n",
           capture_tof_name(capture_mode), capture_mode & 0x03);
    printf("[MESSAGE] iTFS::LITE info_v3 DATA_OUTPUT: %u(0x%04X)\n",
           data_output, data_output);
    printf("                             DEPTH: %s(%u)\n",
           depth_output_name(depth_mode), depth_mode);
    printf("                             AMPLITUDE: %s(%u)\n",
           stream_output_name(amplitude_mode), amplitude_mode);
    printf("                             INTENSITY: %s(%u)\n",
           stream_output_name(intensity_mode), intensity_mode);
    printf("                             CONFIDENCE: %s(%u)\n",
           confidence_output_name(confidence_mode), confidence_mode);
    printf("[MESSAGE] iTFS::LITE info_v3 OTHERS\n");
    printf("                             PERIOD: %u ns\n",
           device->info_v3.capture_period_ns);
    printf("                             EXPOSURE: [ %u, %u, %u ] us\n",
           device->info_v3.capture_shutter[0],
           device->info_v3.capture_shutter[1],
           device->info_v3.capture_shutter[2]);
    printf("                             IP:   %u.%u.%u.%u\n",
           device->info_v3.data_sensor_ip[0],
           device->info_v3.data_sensor_ip[1],
           device->info_v3.data_sensor_ip[2],
           device->info_v3.data_sensor_ip[3]);
    printf("                             DEST: %u.%u.%u.%u:%u (DHCP %s)\n",
           device->info_v3.data_dest_ip[0],
           device->info_v3.data_dest_ip[1],
           device->info_v3.data_dest_ip[2],
           device->info_v3.data_dest_ip[3],
           device->info_v3.data_port,
           capture_filter_name(device->info_v3.data_dhcp_ctrl));
    printf("                             FWL V%u.%u.%u -  ",
           device->info_v3.sensor_fw_ver[2],
           device->info_v3.sensor_fw_ver[1],
           device->info_v3.sensor_fw_ver[0]);
    printf("%s", (const char *)device->info_v3.sensor_fw_time);
    printf(" ");
    printf("%s", (const char *)device->info_v3.sensor_fw_date);
    printf("\n");
}

// Example keyboard input run in seperate thread
static void keyboard_input_run(iTFS::LITE *ilidar) {
    // Wait fot the sensor
    while (ilidar->Ready() != true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Check keyboard input
    while (ilidar->Ready() == true) {
        // Get char input
        char ch = getchar();

        if (ch == 'q' || ch == 'Q') {
            /* Send config packet */
            for (int _i = 0; _i < ilidar->device_cnt; _i++) {
                // Check the info packet was received or not
                if (ilidar->device[_i].status_v3.sensor_sn == ilidar->device[_i].info_v3.sensor_sn) {
                    /* The serial number is matched */

                    /* This example shows how to configure the sensor with API functions */
                    /* See the manual or synchronization docs for details */

                    // Send info_v3 packet to configure the LITE
                    ilidar->device[_i].info_v3.capture_mode = 1;

                    ilidar->device[_i].info_v3.capture_shutter[0] = 800;
                    ilidar->device[_i].info_v3.capture_shutter[1] = 0;
                    ilidar->device[_i].info_v3.capture_shutter[2] = 0;

                    ilidar->device[_i].info_v3.capture_period_ns = 33333333;

                    ilidar->device[_i].info_v3.data_output =
                        iTFS::packet::info_v3_data_output_depth_mm_16bit |
                        iTFS::packet::info_v3_data_output_amplitude_raw_16bit |
                        iTFS::packet::info_v3_data_output_intensity_raw_16bit |
                        iTFS::packet::info_v3_data_output_confidence_raw_16bit;

                    ilidar->Send_config(_i, &(ilidar->device[_i].info_v3));
                    printf("[MESSAGE] iTFS::LITE config(info_v3) packet was sent to D#%d.\n", _i);
                }
            }
        } else if (ch == 'i' || ch == 'I') {
            // Send lock command
            iTFS::packet::cmd_t read_info = {
                0,
            };
            read_info.cmd_id = iTFS::packet::cmd_read_info;
            read_info.cmd_msg = 0;
            ilidar->Send_cmd_to_all(&read_info);
            printf("[MESSAGE] iTFS::LITE cmd_read_info packet was sent.\n");
        } else if (ch == 'w' || ch == 'W') {
            /* Send store command packet */
            for (int _i = 0; _i < ilidar->device_cnt; _i++) {
                // Check the info packet was received or not
                if (ilidar->device[_i].status_v3.sensor_sn == ilidar->device[_i].info_v3.sensor_sn) {
                    /* The serial number is matched */

                    // Send store command
                    iTFS::packet::cmd_t store = {
                        0,
                    };
                    store.cmd_id = iTFS::packet::cmd_store;
                    store.cmd_msg = 0;
                    ilidar->Send_cmd(_i, &store);
                    printf("[MESSAGE] iTFS::LITE cmd_store packet was sent.\n");
                }
            }
        } else if (ch == 'l' || ch == 'L') {
            /* Send lock command packet */
            for (int _i = 0; _i < ilidar->device_cnt; _i++) {
                // Check the info packet was received or not
                if (ilidar->device[_i].status_v3.sensor_sn == ilidar->device[_i].info_v3.sensor_sn) {
                    /* The serial number is matched */

                    // Send lock command
                    iTFS::packet::cmd_t lock = {
                        0,
                    };
                    lock.cmd_id = iTFS::packet::cmd_lock;
                    lock.cmd_msg = ilidar->device[_i].status_v3.sensor_sn;
                    ilidar->Send_cmd(_i, &lock);
                    printf("[MESSAGE] iTFS::LITE cmd_lock packet was sent.\n");
                }
            }
        } else if (ch == 'u' || ch == 'U') {
            /* Send unlock command packet */
            for (int _i = 0; _i < ilidar->device_cnt; _i++) {
                // Check the info packet was received or not
                if (ilidar->device[_i].status_v3.sensor_sn == ilidar->device[_i].info_v3.sensor_sn) {
                    /* The serial number is matched */

                    // Send unlock command
                    iTFS::packet::cmd_t unlock = {
                        0,
                    };
                    unlock.cmd_id = iTFS::packet::cmd_unlock;
                    unlock.cmd_msg = ilidar->device[_i].status_v3.sensor_sn;
                    ilidar->Send_cmd(_i, &unlock);
                    printf("[MESSAGE] iTFS::LITE cmd_unlock packet was sent.\n");
                }
            }
        } else if (ch == 'r' || ch == 'R') {
            /* Send reboot command packet */
            iTFS::packet::cmd_t reboot = {
                0,
            };
            reboot.cmd_id = iTFS::packet::cmd_reboot;
            reboot.cmd_msg = 0;
            ilidar->Send_cmd_to_all(&reboot);
            printf("[MESSAGE] iTFS::LITE cmd_reboot packet was sent.\n");
        } else if (ch == 'p' || ch == 'P') {
            /* Send pause command packet */
            iTFS::packet::cmd_t pause = {
                0,
            };
            pause.cmd_id = iTFS::packet::cmd_pause;
            pause.cmd_msg = 0;
            ilidar->Send_cmd_to_all(&pause);
            printf("[MESSAGE] iTFS::LITE cmd_pause packet was sent.\n");
        } else if (ch == 'o' || ch == 'O') {
            /* Send measure command packet */
            iTFS::packet::cmd_t measure = {
                0,
            };
            measure.cmd_id = iTFS::packet::cmd_measure;
            measure.cmd_msg = 0;
            ilidar->Send_cmd_to_all(&measure);
            printf("[MESSAGE] iTFS::LITE cmd_measure packet was sent.\n");
        }
    }
}
