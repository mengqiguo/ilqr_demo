#pragma once
#include "type.hpp"
#include <Eigen/Dense>
#include <unsupported/Eigen/Splines>


namespace ilqr {

class InitTraj{

public:
    //构造函数
    explicit InitTraj(const std::vector<TrajPoint>& control_points);

    std::vector<TrajPoint> generate_init_traj(int num_output_points = 100)const;

private:
    std::vector<TrajPoint> control_points_;
    // TODO  刚开始直接定义为矩阵/先用vector
};

} // namespace ilqr