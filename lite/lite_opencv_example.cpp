/**
 * @file lite_opencv_example.cpp
 * @brief iTFS-LITE OpenCV example
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

#include <opencv2/opencv.hpp>

#include "../src/ilidar_lite.hpp"

// One reusable image copy per SDK device. The receive callback overwrites it.
static iTFS::lite_img_cpy_t lite_img_data[iTFS::max_device];
static uint16_t lite_data_output[iTFS::max_device];
static constexpr double depth_display_max_mm = 7494.0;

// The queue carries device indexes only; image slots stay in lite_img_data.
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
 * Access must match the data_output mode:
 *   16-bit modes -> get_img_u16_ptr() / get_img_s16_ptr()
 *   8-bit modes  -> get_img_u8_ptr()  / get_img_s8_ptr()
 *
 * Never read an 8-bit slot through u16. A filled u8[240][320] buffer has the
 * same byte span as u16[120][320], so the image will look like half-height
 * data if the wrong union member is used.
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

// Convert any supported LITE depth encoding into an 8-bit display image.
static void scale_depth_display_image(cv::Mat &dst,
                                      const cv::Mat &src,
                                      uint16_t depth_mode,
                                      uint8_t capture_mode,
                                      uint8_t (*depth_image8)[iTFS::lite_max_col]) {

    const double depth_max_mm = (((capture_mode & iTFS::lite_capture_mode_freq_mask) >>
                                 iTFS::lite_capture_mode_freq_pos) == iTFS::lite_capture_mode_freq_f1_single
                                    ? iTFS::depth_f1_max_m
                                    : iTFS::depth_f2_max_m) * 1000.0;

    if (depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
        for (int r = 0; r < iTFS::lite_max_row; r++) {
            for (int c = 0; c < iTFS::lite_max_col; c++) {
                uint16_t depth_mm = iTFS::decode_lite_depth_log8_mm(depth_image8[r][c], capture_mode);
                dst.at<uint8_t>(r, c) = cv::saturate_cast<uint8_t>(depth_mm * 255.0 / depth_display_max_mm);
            }
        }
    } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16 ||
               depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
        src.convertTo(dst, CV_8UC1, depth_max_mm / 65536.0 * 255.0 / depth_display_max_mm);
    } else if (src.type() == CV_8UC1) {
        src.convertTo(dst, CV_8UC1, depth_max_mm / 256.0 * 255.0 / depth_display_max_mm);
    } else {
        src.convertTo(dst, CV_8UC1, 255.0 / depth_display_max_mm);
    }
}

// Example callback declarations. Implementations are kept below main()
// so the user-facing example body appears first.
static void lidar_data_handler(iTFS::lite_device_t *device);
static void status_packet_handler(iTFS::lite_device_t *device);
static void info_packet_handler(iTFS::lite_device_t *device);

// Main example starts here
int main(int argc, char *argv[]) {
    if (iTFS::lite_version() != 0) {
        return -1;
    }

    int key_input = 0;

    // Initialize display buffers
    cv::Mat cv_scaled_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1);
    cv::Mat cv_color_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC3);
    cv::Mat cv_amp_scaled_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1);
    cv::Mat cv_amp_color_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC3);
    cv::Mat cv_intensity_scaled_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1);
    cv::Mat cv_intensity_color_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC3);
    cv::Mat cv_confidence_scaled_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1);
    cv::Mat cv_confidence_color_img = cv::Mat::zeros(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC3);

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
    printf("S/Q/Esc: exit.\n");

    // Main loop starts here
    int recv_device_idx = 0;
    while (true) {
        // Wait for new data
        std::unique_lock<std::mutex> lk(lidar_cv_mutex);
        lidar_cv.wait(lk, [] { return !lidar_q.empty(); });
        recv_device_idx = lidar_q.front();
        lidar_q.pop();

        // Check the main loop underrun
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
        // 1. Decode data_output once.
        // 2. Each image block below declares only the pointer type required by
        //    its selected mode. This keeps 8-bit and 16-bit union access separate.
        // 3. Fold the block for modes/classes you do not use in your application.
        uint16_t data_output = lite_data_output[recv_device_idx];
        uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
        uint16_t amplitude_mode = (data_output & iTFS::packet::info_v3_data_output_amplitude_mask) >> iTFS::packet::info_v3_data_output_amplitude_pos;
        uint16_t intensity_mode = (data_output & iTFS::packet::info_v3_data_output_intensity_mask) >> iTFS::packet::info_v3_data_output_intensity_pos;
        uint16_t confidence_mode = (data_output & iTFS::packet::info_v3_data_output_confidence_mask) >> iTFS::packet::info_v3_data_output_confidence_pos;
        bool amplitude_8bit = amplitude_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
                              amplitude_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
        bool intensity_8bit = intensity_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit ||
                              intensity_mode == iTFS::packet::info_v3_data_output_stream_log_8bit;
        bool confidence_mask1 = confidence_mode == iTFS::packet::info_v3_data_output_confidence_mask_1bit_mode;

        // Depth image block. Open only the mode you use.
        {
            cv::Mat depth_img;
            uint8_t (*depth_image8)[iTFS::lite_max_col] = NULL;
            bool depth_slot_8bit = depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit ||
                                   depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit ||
                                   depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit;

            if (depth_slot_8bit) {
                depth_image8 = iTFS::get_img_u8_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_depth);
                if (depth_image8 != NULL) {
                    depth_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1, &depth_image8[0][0]);
                }
            } else {
                uint16_t (*depth_image16)[iTFS::lite_max_col] =
                    iTFS::get_img_u16_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_depth);
                if (depth_image16 != NULL) {
                    depth_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_16UC1, &depth_image16[0][0]);
                }
            }

            if (depth_img.empty()) {
                printf("[WARNING] iTFS::LITE There is no output depth image.\n");
                continue;
            }

            scale_depth_display_image(cv_scaled_img,
                                      depth_img,
                                      depth_mode,
                                      lite_img_data[recv_device_idx].mode,
                                      depth_image8);
            cv::applyColorMap(cv_scaled_img, cv_color_img, cv::COLORMAP_JET);

            // Set the openCV window name
            std::string window_name = "iTFS-LITE D" +
                                      std::to_string(recv_device_idx) +
                                      " " +
                                      std::to_string(lite->device[recv_device_idx].ip[0]) + "." +
                                      std::to_string(lite->device[recv_device_idx].ip[1]) + "." +
                                      std::to_string(lite->device[recv_device_idx].ip[2]) + "." +
                                      std::to_string(lite->device[recv_device_idx].ip[3]) + ":" +
                                      std::to_string(lite->device[recv_device_idx].port) +
                                      " DEPTH";

            // Check the viewer is closed
            static int cv_window = 0;
            if (cv_window == 0) {
                // Create window
                cv::namedWindow(window_name, cv::WINDOW_NORMAL);
                cv_window = 1;
            }

            // Call imshow
            cv::imshow(window_name, cv_color_img);
        }

        // Amplitude image block.
        {
            cv::Mat amp_img;

            if (amplitude_8bit) {
                uint8_t (*amplitude_image8)[iTFS::lite_max_col] =
                    iTFS::get_img_u8_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_amplitude);
                if (amplitude_image8 != NULL) {
                    amp_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1, &amplitude_image8[0][0]);
                }
            } else {
                uint16_t (*amplitude_image16)[iTFS::lite_max_col] =
                    iTFS::get_img_u16_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_amplitude);
                if (amplitude_image16 != NULL) {
                    amp_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_16UC1, &amplitude_image16[0][0]);
                }
            }

            if (!amp_img.empty()) {
                if (amplitude_8bit) {
                    if (amplitude_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit) {
                        // A/I lin8 transmits the raw high byte. The 63D useful range is about 14 bits,
                        // so expand 0..0x3F to the display range.
                        amp_img.convertTo(cv_amp_scaled_img, CV_8UC1, 255.0 / 0x3F);
                    } else {
                        cv_amp_scaled_img = amp_img;
                    }
                } else {
                    amp_img.convertTo(cv_amp_scaled_img, CV_8UC1, 255.0 / 0x3FFF);
                }
                cv::applyColorMap(cv_amp_scaled_img, cv_amp_color_img, cv::COLORMAP_JET);

                std::string amp_window_name = "iTFS-LITE D" +
                                              std::to_string(recv_device_idx) +
                                              " " +
                                              std::to_string(lite->device[recv_device_idx].ip[0]) + "." +
                                              std::to_string(lite->device[recv_device_idx].ip[1]) + "." +
                                              std::to_string(lite->device[recv_device_idx].ip[2]) + "." +
                                              std::to_string(lite->device[recv_device_idx].ip[3]) + ":" +
                                              std::to_string(lite->device[recv_device_idx].port) +
                                              " AMPLITUDE";

                static int cv_amp_window = 0;
                if (cv_amp_window == 0) {
                    cv::namedWindow(amp_window_name, cv::WINDOW_NORMAL);
                    cv_amp_window = 1;
                }

                cv::imshow(amp_window_name, cv_amp_color_img);
            }
        }

        // Intensity image block.
        {
            cv::Mat intensity_img;

            if (intensity_8bit) {
                uint8_t (*intensity_image8)[iTFS::lite_max_col] =
                    iTFS::get_img_u8_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_intensity);
                if (intensity_image8 != NULL) {
                    intensity_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1, &intensity_image8[0][0]);
                }
            } else {
                uint16_t (*intensity_image16)[iTFS::lite_max_col] =
                    iTFS::get_img_u16_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_intensity);
                if (intensity_image16 != NULL) {
                    intensity_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_16UC1, &intensity_image16[0][0]);
                }
            }

            if (!intensity_img.empty()) {
                if (intensity_8bit) {
                    if (intensity_mode == iTFS::packet::info_v3_data_output_stream_lin_8bit) {
                        // A/I lin8 transmits the raw high byte. The 63D useful range is about 14 bits,
                        // so expand 0..0x3F to the display range.
                        intensity_img.convertTo(cv_intensity_scaled_img, CV_8UC1, 255.0 / 0x3F);
                    } else {
                        cv_intensity_scaled_img = intensity_img;
                    }
                } else {
                    intensity_img.convertTo(cv_intensity_scaled_img, CV_8UC1, 255.0 / 0x3FFF);
                }
                cv::applyColorMap(cv_intensity_scaled_img, cv_intensity_color_img, cv::COLORMAP_JET);

                std::string intensity_window_name = "iTFS-LITE D" +
                                                    std::to_string(recv_device_idx) +
                                                    " " +
                                                    std::to_string(lite->device[recv_device_idx].ip[0]) + "." +
                                                    std::to_string(lite->device[recv_device_idx].ip[1]) + "." +
                                                    std::to_string(lite->device[recv_device_idx].ip[2]) + "." +
                                                    std::to_string(lite->device[recv_device_idx].ip[3]) + ":" +
                                                    std::to_string(lite->device[recv_device_idx].port) +
                                                    " INTENSITY";

                static int cv_intensity_window = 0;
                if (cv_intensity_window == 0) {
                    cv::namedWindow(intensity_window_name, cv::WINDOW_NORMAL);
                    cv_intensity_window = 1;
                }

                cv::imshow(intensity_window_name, cv_intensity_color_img);
            }
        }

        // Confidence image block.
        {
            cv::Mat confidence_img;

            if (confidence_mask1) {
                uint8_t (*confidence_image8)[iTFS::lite_max_col] =
                    iTFS::get_img_u8_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_confidence);
                if (confidence_image8 != NULL) {
                    confidence_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_8UC1, &confidence_image8[0][0]);
                }
            } else {
                uint16_t (*confidence_image16)[iTFS::lite_max_col] =
                    iTFS::get_img_u16_ptr(&lite_img_data[recv_device_idx], iTFS::lite_img_confidence);
                if (confidence_image16 != NULL) {
                    confidence_img = cv::Mat(iTFS::lite_max_row, iTFS::lite_max_col, CV_16UC1, &confidence_image16[0][0]);
                }
            }

            if (!confidence_img.empty()) {
                if (confidence_mask1) {
                    // Confidence mask1 is binary data. Show 0 as black and valid pixels as white.
                    confidence_img.convertTo(cv_confidence_scaled_img, CV_8UC1, 255.0);
                } else {
                    confidence_img.convertTo(cv_confidence_scaled_img, CV_8UC1, 255.0 / 65535);
                }
                if (confidence_mask1) {
                    cv::cvtColor(cv_confidence_scaled_img, cv_confidence_color_img, cv::COLOR_GRAY2BGR);
                } else {
                    cv::applyColorMap(cv_confidence_scaled_img, cv_confidence_color_img, cv::COLORMAP_JET);
                }

                std::string confidence_window_name = "iTFS-LITE D" +
                                                     std::to_string(recv_device_idx) +
                                                     " " +
                                                     std::to_string(lite->device[recv_device_idx].ip[0]) + "." +
                                                     std::to_string(lite->device[recv_device_idx].ip[1]) + "." +
                                                     std::to_string(lite->device[recv_device_idx].ip[2]) + "." +
                                                     std::to_string(lite->device[recv_device_idx].ip[3]) + ":" +
                                                     std::to_string(lite->device[recv_device_idx].port) +
                                                     " CONFIDENCE";

                static int cv_confidence_window = 0;
                if (cv_confidence_window == 0) {
                    cv::namedWindow(confidence_window_name, cv::WINDOW_NORMAL);
                    cv_confidence_window = 1;
                }

                cv::imshow(confidence_window_name, cv_confidence_color_img);
            }
        }

        key_input = cv::waitKey(10);

        // Check window property
        if (key_input == 's' || key_input == 'S' || key_input == 'q' || key_input == 'Q' || key_input == 27) {
            break;
        }
        /**************************************/
    }

    // Destroy all openCV windows
    cv::destroyAllWindows();
    cv::waitKey(10);

    // Stop and delete iTFS LITE class
    delete lite;
    printf("[MESSAGE] iTFS::LITE has been deleted.\n");

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

// Image callback: copy active SDK-owned slots, then wake the UI loop.
static void lidar_data_handler(iTFS::lite_device_t *device) {
    std::lock_guard<std::mutex> lk(lidar_cv_mutex);
    iTFS::lite_img_t *src = &device->data;
    iTFS::lite_img_cpy_t *dst = &lite_img_data[device->idx];

    // Call metadata copy helper function of the lite data
    iTFS::copy_img_metadata(dst, src);

    // Deep-copy the lite data
    memcpy((void *)dst->data,
           (const void *)src->data,
           sizeof(iTFS::lite_img_slot_t) * dst->data_class_count);
    lite_data_output[device->idx] = device->info_v3.data_output;

    // Notify the reception to the main thread
    int idx = device->idx;
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
