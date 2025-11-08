/*
 * @Author: 在百慕大钓鱼的人 mayuzhuonor@hotmail.com
 * @Date: 2025-10-30 16:37:04
 * @LastEditors: 在百慕大钓鱼的人 mayuzhuonor@hotmail.com
 * @LastEditTime: 2025-10-30 17:07:58
 * @FilePath: /sp_vision_25/io/wit_imu/wit_imu.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef WIT_IMU_H
#define WIT_IMU_H

#include <string>
#include <Eigen/Geometry>

namespace io {
class WIT_IMU{
public:
    WIT_IMU(std::string dev_name_, int rate_ = 115200);
    ~WIT_IMU();

    Eigen::Quaterniond get_data();

protected:
    std::string dev_name;
    int rate;
};
}

#endif // WIT_IMU_H