#include "thermal_camera.h"
#include <opencv2/highgui.hpp>

// Variables
static uint16_t eeMLX90640[832];
static paramsMLX90640 mlx90640;


mlx_thermal_cam::mlx_thermal_cam()
{
    MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
    MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);
    MLX90640_SetRefreshRate(MLX_I2C_ADDR, FRAMES_PER_SECOND);
    MLX90640_SetChessMode(MLX_I2C_ADDR);

    MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
    MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
    int refresh_rate = MLX90640_GetRefreshRate(MLX_I2C_ADDR);
    float fps = convert_refresh_to_fps(refresh_rate);
    ROS_INFO("EE Dumped. Refresh rate is %ffps", fps);

    ROS_INFO("Configured thermal camera!");
}

mlx_thermal_cam::~mlx_thermal_cam()
{

}

cv::Mat mlx_thermal_cam::get_frame()
{
    cv::Mat mat;
    mat.create(FRAME_ROWS, FRAME_COLUMNS, CV_8U);

    float eTa;
    static float mlx90640To[768];
    float emissivity = 1;
    uint16_t frame[834];

    MLX90640_GetFrameData(MLX_I2C_ADDR, frame);
    // MLX90640_InterpolateOutliers(frame, eeMLX90640);
    eTa = MLX90640_GetTa(frame, &mlx90640);
    MLX90640_CalculateTo(frame, &mlx90640, emissivity, eTa, mlx90640To);
    MLX90640_BadPixelsCorrection((&mlx90640)->brokenPixels, mlx90640To, 1, &mlx90640);
    MLX90640_BadPixelsCorrection((&mlx90640)->outlierPixels, mlx90640To, 1, &mlx90640);

    for(int x = 0; x < 32; x++){
      for(int y = 0; y < 24; y++){
        // Clamp pixels in range 0 to 255C.
        float pixel_temperature_c = mlx90640To[32 * (23-y) + x];
        // uchar is an openCV type.
        uchar pixel_c = 0;
        if(pixel_temperature_c > 255) {
          pixel_c = 255;
        } else if(pixel_temperature_c < 0.0) {
          pixel_c = 0;
        } else {
          pixel_c = static_cast<uchar>(pixel_temperature_c);
        }
        // Update the pixel data with the temperature value.
        mat.at<uchar>(x, y) = pixel_c;
      }  // y
    }  // x

    return mat;
}

float mlx_thermal_cam::convert_refresh_to_fps(int refresh_rate)
{
    float fps = 0.0;
    switch(refresh_rate) {
        case 0x00:
        fps = 0.5;
        break;
        case 0x01:
        fps = 1.0;
        break;
        case 0x02:
        fps = 2;
        break;
        case 0x03:
        fps = 4;
        break;
        case 0x04:
        fps = 8;
        break;
        case 0x05:
        fps = 16;
        break;
        case 0x06:
        fps = 32;
        break;
        case 0x07:
        fps = 64;
        break;
        default:
        ROS_WARN("Invalid refresh_rate %d", refresh_rate);
    }
    return fps;
}
