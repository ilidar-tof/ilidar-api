/**
 * @file lite_pcl_example.cpp
 * @brief iTFS-LITE PCL example
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

#include <Eigen/Dense>

#pragma warning(disable : 4996)
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>

#include "../src/ilidar_lite.hpp"

// PCL point cloud with XYZ RGB A
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scan[iTFS::max_device];

// Point cloud height colormap example
static float point_cloud_height_max = 1.5f;
static float point_cloud_height_min = -1.5f;

// Color converter for pointcloud
auto hsv2rgb = [](float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
    int i = int(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    float rf, gf, bf;
    switch (i % 6) {
    case 0:
        rf = v;
        gf = t;
        bf = p;
        break;
    case 1:
        rf = q;
        gf = v;
        bf = p;
        break;
    case 2:
        rf = p;
        gf = v;
        bf = t;
        break;
    case 3:
        rf = p;
        gf = q;
        bf = v;
        break;
    case 4:
        rf = t;
        gf = p;
        bf = v;
        break;
    case 5:
        rf = v;
        gf = p;
        bf = q;
        break;
    }
    r = static_cast<uint8_t>(rf * 255);
    g = static_cast<uint8_t>(gf * 255);
    b = static_cast<uint8_t>(bf * 255);
};

static inline void colorize_point(pcl::PointXYZRGBA &point) {
    float heightLevel = (point.z - point_cloud_height_min) / (point_cloud_height_max - point_cloud_height_min);
    if (heightLevel > 0.99f) {
        heightLevel = 1.0f;
    }
    if (heightLevel < 0.01f) {
        heightLevel = 0.0f;
    }

    uint8_t r, g, b;
    hsv2rgb(0.66f * (1.0f - heightLevel), 1.0f, 1.0f, r, g, b);

    point.r = r;
    point.g = g;
    point.b = b;
    point.a = 0xFF;
}

// Transform a float32 meter depth image to a point cloud.
void transform_depth_float(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dst,
                           float depth_img[iTFS::lite_max_row][iTFS::lite_max_col],
                           iTFS::lite_vec3d_t *map) {
    for (int _i = 0; _i < iTFS::lite_max_col; _i++) {
        for (int _j = 0; _j < iTFS::lite_max_row; _j++) {
            float depth = depth_img[_j][_i];
            float depht_vector_x = map->dir[0][_j][_i];
            float depht_vector_y = map->dir[1][_j][_i];
            float depht_vector_z = map->dir[2][_j][_i];

            int idx = _j * iTFS::lite_max_col + _i;
            dst.get()->points[idx].x = depth * depht_vector_z;
            dst.get()->points[idx].y = -depth * depht_vector_x;
            dst.get()->points[idx].z = -depth * depht_vector_y;

            colorize_point(dst.get()->points[idx]);
        }
    }
}

// Transform float32 native XYZ meter images to a point cloud.
void transform_xyz_float(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr dst,
                         float x_img[iTFS::lite_max_row][iTFS::lite_max_col],
                         float y_img[iTFS::lite_max_row][iTFS::lite_max_col],
                         float z_img[iTFS::lite_max_row][iTFS::lite_max_col]) {
    for (int _i = 0; _i < iTFS::lite_max_col; _i++) {
        for (int _j = 0; _j < iTFS::lite_max_row; _j++) {
            int idx = _j * iTFS::lite_max_col + _i;
            dst.get()->points[idx].x = z_img[_j][_i];
            dst.get()->points[idx].y = -x_img[_j][_i];
            dst.get()->points[idx].z = -y_img[_j][_i];

            colorize_point(dst.get()->points[idx]);
        }
    }
}

// Draw basic grid
void drawGrid(pcl::visualization::PCLVisualizer::Ptr p, int x, int y, int z) {
    for (int i = 0; i < (2 * x + 1); i++) {
        char name[32] = "L00";
        name[2] = 0x21 + i;
        pcl::PointXYZ p1, p2;
        p1.x = i - x;
        p1.y = y;
        p1.z = z;

        p2.x = i - x;
        p2.y = -y;
        p2.z = z;

        p->addLine(p1, p2, 0.3, 0.3, 0.3, name);
    }
    for (int i = 0; i < (2 * y + 1); i++) {
        char name[32] = "L10";
        name[2] = 0x21 + i;
        pcl::PointXYZ p1, p2;
        p1.x = x;
        p1.y = i - y;
        p1.z = z;

        p2.x = -x;
        p2.y = i - y;
        p2.z = z;

        p->addLine(p1, p2, 0.3, 0.3, 0.3, name);
    }
}

// One reusable image copy and point workspace per SDK device.
static iTFS::lite_img_cpy_t lite_img_data[iTFS::max_device];
static float lite_depth_float_data[iTFS::max_device][iTFS::lite_max_row][iTFS::lite_max_col];
static float lite_x_float_data[iTFS::max_device][iTFS::lite_max_row][iTFS::lite_max_col];
static float lite_y_float_data[iTFS::max_device][iTFS::lite_max_row][iTFS::lite_max_col];
static float lite_z_float_data[iTFS::max_device][iTFS::lite_max_row][iTFS::lite_max_col];

// Return an enabled image slot, or NULL when that class is absent.
static iTFS::lite_img_slot_t *get_img_slot(iTFS::lite_img_cpy_t *data, uint8_t img_class) {
    if (img_class >= iTFS::lite_img_class_count) {
        return NULL;
    }
    if (data->img_offset[img_class] < 0) {
        return NULL;
    }

    return &data->data[data->img_offset[img_class] / iTFS::lite_max_row];
}

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
 * Access must match the data_output mode. The image slot is a union:
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

    // Create the viewer
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("PCL Viewer Example"));

    // Create the point cloud holder
    int point_cloud_size = iTFS::lite_max_col * iTFS::lite_max_row;
    for (int _i = 0; _i < iTFS::max_device; _i++) {
        scan[_i] = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>);
        scan[_i].get()->points.resize(point_cloud_size);
    }

    // Draw grid
    drawGrid(viewer, 20, 20, -1);
    viewer->addCoordinateSystem(1.0);

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
        // 2. Each mode block below declares only the union member it actually
        //    reads. Keep u8 and u16 access separate.
        // 3. Fold the mode blocks you do not use in your application.
        uint16_t data_output = lite->device[recv_device_idx].info_v3.data_output;
        uint16_t depth_mode = data_output & iTFS::packet::info_v3_data_output_depth_mask;
        const bool f1 = (lite_img_data[recv_device_idx].mode & iTFS::lite_capture_mode_freq_mask) ==
                        (iTFS::lite_capture_mode_freq_f1_single << iTFS::lite_capture_mode_freq_pos);
        const float depth_raw_q16_to_m = (f1 ? iTFS::depth_f1_max_m : iTFS::depth_f2_max_m) / 65536.0f;
        const float depth_lin8_to_m = (f1 ? iTFS::depth_f1_max_m : iTFS::depth_f2_max_m) / 256.0f;
        const float xyz_raw_q15_to_m = (f1 ? iTFS::depth_f1_max_m : iTFS::depth_f2_max_m) / 32768.0f;
        const float xyz_lin8_to_m = (f1 ? iTFS::depth_f1_max_m : iTFS::depth_f2_max_m) / 128.0f;

        // Convert depth image to point cloud. Depth-only modes come first;
        // XYZ modes are handled after that as native point image outputs.
        if (lite_img_data[recv_device_idx].depth_on) {
            iTFS::lite_img_slot_t *depth_slot =
                get_img_slot(&lite_img_data[recv_device_idx], iTFS::lite_img_depth);

            if (depth_slot == NULL) {
                printf("[WARNING] iTFS::LITE Invalid depth image slot.\n");
                continue;
            }

            if (depth_mode == iTFS::packet::info_v3_data_output_depth_mm_16bit) {
                // depth_mm_16bit block
                // 16-bit millimeter output: cast to a user-side float32 meter image.
                uint16_t (*depth_image16)[iTFS::lite_max_col] = depth_slot->u16;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_depth_float_data[recv_device_idx][r][c] = static_cast<float>(depth_image16[r][c]) * 0.001f;
                    }
                }

                transform_depth_float(scan[recv_device_idx],
                                      lite_depth_float_data[recv_device_idx],
                                      &lite->device[recv_device_idx].reconstruction_map);
            } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_raw_q16) {
                // depth_raw_q16 block
                // 16-bit raw Q16 output: access the union slot as u16, then
                // cast to a user-side float32 meter image.
                uint16_t (*depth_image16)[iTFS::lite_max_col] = depth_slot->u16;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_depth_float_data[recv_device_idx][r][c] = static_cast<float>(depth_image16[r][c]) * depth_raw_q16_to_m;
                    }
                }

                transform_depth_float(scan[recv_device_idx],
                                      lite_depth_float_data[recv_device_idx],
                                      &lite->device[recv_device_idx].reconstruction_map);
            } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_lin_8bit) {
                // depth_lin_8bit block
                // 8-bit linear output: access the union slot as u8, then
                // cast to a user-side float32 meter image.
                uint8_t (*depth_image8)[iTFS::lite_max_col] = depth_slot->u8;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_depth_float_data[recv_device_idx][r][c] = static_cast<float>(depth_image8[r][c]) * depth_lin8_to_m;
                    }
                }

                transform_depth_float(scan[recv_device_idx],
                                      lite_depth_float_data[recv_device_idx],
                                      &lite->device[recv_device_idx].reconstruction_map);
            } else if (depth_mode == iTFS::packet::info_v3_data_output_depth_log_8bit) {
                // depth_log_8bit block
                // 8-bit LOG output: access the union slot as u8 only, then
                // decode the LUT to a user-side float32 meter image.
                uint8_t (*depth_image8)[iTFS::lite_max_col] = depth_slot->u8;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_depth_float_data[recv_device_idx][r][c] =
                            static_cast<float>(iTFS::decode_lite_depth_log8_mm(
                                depth_image8[r][c], lite_img_data[recv_device_idx].mode)) * 0.001f;
                    }
                }

                transform_depth_float(scan[recv_device_idx],
                                      lite_depth_float_data[recv_device_idx],
                                      &lite->device[recv_device_idx].reconstruction_map);
            } else {
                printf("[WARNING] iTFS::LITE Unsupported depth output for PCL reconstruction.\n");
                continue;
            }
        } else if (lite_img_data[recv_device_idx].xyz_on) {
            // XYZ image block. This path is used only by xyz_* depth modes.
            iTFS::lite_img_slot_t *z_slot =
                get_img_slot(&lite_img_data[recv_device_idx], iTFS::lite_img_depth);
            iTFS::lite_img_slot_t *x_slot =
                get_img_slot(&lite_img_data[recv_device_idx], iTFS::lite_img_point_x);
            iTFS::lite_img_slot_t *y_slot =
                get_img_slot(&lite_img_data[recv_device_idx], iTFS::lite_img_point_y);

            if (z_slot == NULL || x_slot == NULL || y_slot == NULL) {
                printf("[WARNING] iTFS::LITE Invalid XYZ image slot.\n");
                continue;
            }

            if (depth_mode == iTFS::packet::info_v3_data_output_xyz_mm_16bit) {
                // xyz_mm_16bit block
                uint16_t (*z_u16)[iTFS::lite_max_col] = z_slot->u16;
                int16_t (*x_s16)[iTFS::lite_max_col] = x_slot->s16;
                int16_t (*y_s16)[iTFS::lite_max_col] = y_slot->s16;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_x_float_data[recv_device_idx][r][c] = static_cast<float>(x_s16[r][c]) * 0.001f;
                        lite_y_float_data[recv_device_idx][r][c] = static_cast<float>(y_s16[r][c]) * 0.001f;
                        lite_z_float_data[recv_device_idx][r][c] = static_cast<float>(z_u16[r][c]) * 0.001f;
                    }
                }
            } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_raw_q15_q16) {
                // xyz_raw_q15_q16 block
                uint16_t (*z_u16)[iTFS::lite_max_col] = z_slot->u16;
                int16_t (*x_s16)[iTFS::lite_max_col] = x_slot->s16;
                int16_t (*y_s16)[iTFS::lite_max_col] = y_slot->s16;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_x_float_data[recv_device_idx][r][c] = static_cast<float>(x_s16[r][c]) * xyz_raw_q15_to_m;
                        lite_y_float_data[recv_device_idx][r][c] = static_cast<float>(y_s16[r][c]) * xyz_raw_q15_to_m;
                        lite_z_float_data[recv_device_idx][r][c] = static_cast<float>(z_u16[r][c]) * depth_raw_q16_to_m;
                    }
                }
            } else if (depth_mode == iTFS::packet::info_v3_data_output_xyz_lin_8bit) {
                // xyz_lin_8bit block
                uint8_t (*z_u8)[iTFS::lite_max_col] = z_slot->u8;
                int8_t (*x_s8)[iTFS::lite_max_col] = x_slot->s8;
                int8_t (*y_s8)[iTFS::lite_max_col] = y_slot->s8;

                for (int r = 0; r < iTFS::lite_max_row; r++) {
                    for (int c = 0; c < iTFS::lite_max_col; c++) {
                        lite_x_float_data[recv_device_idx][r][c] = static_cast<float>(x_s8[r][c]) * xyz_lin8_to_m;
                        lite_y_float_data[recv_device_idx][r][c] = static_cast<float>(y_s8[r][c]) * xyz_lin8_to_m;
                        lite_z_float_data[recv_device_idx][r][c] = static_cast<float>(z_u8[r][c]) * depth_lin8_to_m;
                    }
                }
            } else {
                printf("[WARNING] iTFS::LITE Unsupported XYZ output for PCL reconstruction.\n");
                continue;
            }

            transform_xyz_float(scan[recv_device_idx],
                                lite_x_float_data[recv_device_idx],
                                lite_y_float_data[recv_device_idx],
                                lite_z_float_data[recv_device_idx]);
        } else {
            // There is no output depth image..
            printf("[WARNING] iTFS::LITE There is no output depth image.\n");
            continue;
        }

        // Apply spacing for multiple lidars
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.translation() << 0.0f, (recv_device_idx * 3), 0.0f; // 3 m spacing
        pcl::transformPointCloud(*scan[recv_device_idx], *scan[recv_device_idx], transform);

        // Update the viewer
        std::string cloud_name = "scan" + std::to_string(recv_device_idx);
        if (!viewer->updatePointCloud(scan[recv_device_idx], cloud_name)) {
            viewer->addPointCloud(scan[recv_device_idx], cloud_name);
            viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloud_name);
        }

        // Sleep for display
        viewer->spinOnce(10);

        // Check the viewer is closed
        if (viewer->wasStopped()) {
            break;
        }
    }

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

// Image callback: copy active SDK-owned slots, then wake the viewer loop.
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
