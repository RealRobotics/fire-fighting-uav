/* Copyright 2020, University of Leeds.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

Based on the ROS tutorial:
http://wiki.ros.org/image_transport/Tutorials/PublishingImages
and example code in the MLX90640 pimoroni library:
https://github.com/AndyBlightLeeds/mlx90640-library/blob/master/examples/src/test.cpp
*/


#include <chrono>
#include <thread>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "MLX90640_API.h"

#define FRAME_COLUMNS (24)
#define FRAME_ROWS (32)
#define IMAGE_SIZE (FRAME_COLUMNS * FRAME_ROWS)
#define NODE_NAME "mlx90640"
#define MLX_I2C_ADDR (0x33)

// Valid frame rates are 1, 2, 4, 8, 16, 32 and 64
// Use 32 FPS.
#define FRAMES_PER_SECOND (0b110)

// Despite the framerate being ostensibly FPS hz
// The frame is often not ready in time
// This offset is added to the FRAME_TIME_MICROS
// to account for this.
#define OFFSET_MICROS (850)


float convert_refresh_to_fps(int refresh_rate) {
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

int main(int argc, char *argv[]){
  // Setup ROS node and advertise image.
  ros::init(argc, argv, NODE_NAME);
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("thermal_camera_mlx90640/image", 1);

  ROS_INFO("Starting...");
  // Variables
  static uint16_t eeMLX90640[832];
  float emissivity = 1;
  uint16_t frame[834];
  float eTa;

  // Initialise device.
  MLX90640_SetDeviceMode(MLX_I2C_ADDR, 0);
  MLX90640_SetSubPageRepeat(MLX_I2C_ADDR, 0);
  MLX90640_SetRefreshRate(MLX_I2C_ADDR, FRAMES_PER_SECOND);
  MLX90640_SetChessMode(MLX_I2C_ADDR);
  ROS_INFO("Configured...");

  paramsMLX90640 mlx90640;
  MLX90640_DumpEE(MLX_I2C_ADDR, eeMLX90640);
  MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  int refresh_rate = MLX90640_GetRefreshRate(MLX_I2C_ADDR);
  float fps = convert_refresh_to_fps(refresh_rate);
  ROS_INFO("EE Dumped. Refresh rate is %fpfs", fps);

  // Prepare for loop.
  cv::Mat mat;
  mat.create(FRAME_ROWS, FRAME_COLUMNS, CV_8U);
  sensor_msgs::ImagePtr msg;
  static float mlx90640To[768];
  while (nh.ok()) {
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
    // Publish black and white image, 8 bits per pixel.
    msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", mat).toImageMsg();
    pub.publish(msg);
    // Let other threads run.
    ros::spinOnce();
  }  // while
  return 0;
}
