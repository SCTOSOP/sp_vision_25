#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  std::pair<double, std::vector<cv::Point2f>> armor_reprojection_error(const Armor & armor, double yaw, const double & inclined, bool useless) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

class OneDKalmanFilter {
  private:
      cv::KalmanFilter kf;
      cv::Mat measurement;
      
  public:
      // 构造函数
      OneDKalmanFilter(double processNoise = 1e-5, double measurementNoise = 1e-1, double initialError = 1.0) {
          // 初始化卡尔曼滤波器
          // 状态维度: 2 (位置和速度)
          // 测量维度: 1 (只测量位置)
          // 控制维度: 0 (无控制输入)
          kf.init(2, 1, 0);
          
          // 状态转移矩阵 (假设匀速运动模型)
          // [1, dt]
          // [0, 1]
          kf.transitionMatrix = (cv::Mat_<float>(2, 2) << 1, 1, 0, 1);
          
          // 测量矩阵 (我们只能测量位置)
          kf.measurementMatrix = (cv::Mat_<float>(1, 2) << 1, 0);
          
          // 过程噪声协方差矩阵
          kf.processNoiseCov = (cv::Mat_<float>(2, 2) << 
              processNoise, 0,
              0, processNoise);
          
          // 测量噪声协方差矩阵
          kf.measurementNoiseCov = (cv::Mat_<float>(1, 1) << measurementNoise);
          
          // 后验误差协方差矩阵
          kf.errorCovPost = (cv::Mat_<float>(2, 2) << 
              initialError, 0,
              0, initialError);
          
          // 状态后验估计 (初始化为0)
          kf.statePost = (cv::Mat_<float>(2, 1) << 0, 0);
          
          measurement = cv::Mat::zeros(1, 1, CV_32F);
      }
      
      // 预测步骤
      float predict() {
          cv::Mat prediction = kf.predict();
          return prediction.at<float>(0); // 返回预测的位置
      }
      
      // 更新步骤
      float update(float measured_value) {
          measurement.at<float>(0) = measured_value;
          cv::Mat corrected = kf.correct(measurement);
          return corrected.at<float>(0); // 返回修正后的位置
      }
      
      // 获取当前状态
      void getState(float& position, float& velocity) {
          position = kf.statePost.at<float>(0);
          velocity = kf.statePost.at<float>(1);
      }
      
      // 设置初始状态
      void setInitialState(float initial_position, float initial_velocity = 0) {
          kf.statePost = (cv::Mat_<float>(2, 1) << initial_position, initial_velocity);
      }
  };

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP