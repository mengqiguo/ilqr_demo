#pragma once
#include <Eigen/Dense>
#include <utility> 
#include "type.hpp"

namespace ilqr {

class Bicycle_model{
public:
    Bicycle_model(double L, double dt);//形参
    Car_state trans_state(const Car_state& cur_state, const Control& cur_u)const;
    
    std::pair<Eigen::MatrixXd, Eigen::MatrixXd>get_init_traj(int N,const Car_state& state,const Control&   control) const;//传参要引用

    //状态求导
    Eigen::MatrixXd jacobian_f_x(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& u) const;
    Eigen::MatrixXd jacobian_f_u(const Eigen::VectorXd& x,
                                 const Eigen::VectorXd& u) const;
    
    //单步动力学应用
    Eigen::VectorXd dynamics(const Eigen::VectorXd& x,
                             const Eigen::VectorXd& u) const;
private:
    double L_;//轴距
    double dt_;//采样时间
};
} // namespace ilqr
