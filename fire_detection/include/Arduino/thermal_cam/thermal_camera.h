#ifndef THERMAL_CAMERA_H
#define THERMAL_CAMERA_H

#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include "mlx90640-library/headers/MLX90640_API.h"

#define FRAME_COLUMNS (24)
#define FRAME_ROWS (32)
#define IMAGE_SIZE (FRAME_COLUMNS * FRAME_ROWS)
#define MLX_I2C_ADDR (0x33)

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// Use 32 FPS.
#define FRAMES_PER_SECOND (0b110)

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS (850)

class mlx_thermal_cam
{
public:
    mlx_thermal_cam();
    ~mlx_thermal_cam();

    cv::Mat get_frame();

private:
    float convert_refresh_to_fps(int refresh_rate);

};
#endif
