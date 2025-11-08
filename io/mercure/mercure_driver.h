/*
 * @Author: 在百慕大钓鱼的人 mayuzhuonor@hotmail.com
 * @Date: 2025-10-30 15:16:00
 * @LastEditors: 在百慕大钓鱼的人 mayuzhuonor@hotmail.com
 * @LastEditTime: 2025-10-30 15:41:52
 * @FilePath: /sp_vision_25/io/mercure/mercure_driver.h
 * @Description: 
 */
#ifndef MERCURE_DRIVER_H
#define MERCURE_DRIVER_H

#include "../camera.hpp"
#include "GxIAPI.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <string>
#include <chrono>

namespace io{

#define ACQ_TRANSFER_SIZE       (64 * 1024)
#define ACQ_TRANSFER_NUMBER_URB  64

#define ACQ_FRAME_WIDTH   640
#define ACQ_FRAME_HEIGHT  480

#define ACQ_FRAME_TYPE    CV_8UC3

#define ACQ_BUFFER_NUM 3

struct Param
{
  int exp_auto_;
  int w_auto_;
  int gain_auto_;
  double exp_time_; // us
  double w_red_;
  double w_green_;
  double w_blue_;
  double gain_;
};

class MercureDriver : public CameraBase
{
  GX_STATUS     status_;
  GX_DEV_HANDLE device_;

  PGX_FRAME_BUFFER pFrameBuffer_;
  uint8_t* rgbImagebuf_;   
  Param param_;

public:
  explicit MercureDriver();
  GX_STATUS init_sdk();
  void GetVision();
  void operator >> (cv::Mat& Image); // API
  // void operator >> (Mat_time& Image); // API
  void LoadParam(const std::string & file_name = "/home/admin/vision_26/Configure/Settings.xml");
  void resetParam(const std::string & file_name = "/home/admin/vision_26/Configure/Settings.xml");   // 重置相机参数

  static void CreateMat(cv::Mat& mat_image);

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;

  ~MercureDriver();
};

} // namespace camera

#endif // MERCURE_DRIVER_H