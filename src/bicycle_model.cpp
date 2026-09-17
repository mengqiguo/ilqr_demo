#include "bicycle_model.hpp"
#include "type.hpp"
#include <utility> 
#include <cmath>

namespace ilqr {
    
    
//构造函数初始化成员变量
Bicycle_model::Bicycle_model(double L, double dt):L_(L), dt_(dt){}

//状态转移函数实现
Car_state Bicycle_model::trans_state(const Car_state& cur_state, const Control& cur_u)const{
    Car_state next_state;
    next_state.x = cur_state.x + cur_state.v * cos(cur_state.theta) * dt_;
    next_state.y = cur_state.y + cur_state.v * sin(cur_state.theta) * dt_;
    next_state.theta = cur_state.theta + (cur_state.v / L_) * tan(cur_u.delta) * dt_;
    next_state.v = cur_state.v + cur_u.a * dt_;
    return next_state;
}

//与 trans_state 完全一致的向量版（保证导数与 rollout 一致）
//TODO： 合并状态转移部分
Eigen::VectorXd Bicycle_model::dynamics(const Eigen::VectorXd& x,
                                        const Eigen::VectorXd& u) const
{
    double px    = x(0);
    double py    = x(1);
    double theta = x(2);
    double v     = x(3);
    double a     = u(0);
    double delta = u(1);

    Eigen::VectorXd xn(4);
    xn(0) = px + v * std::cos(theta) * dt_;
    xn(1) = py + v * std::sin(theta) * dt_;
    xn(2) = theta + (v / L_) * std::tan(delta) * dt_;
    xn(3) = v + a * dt_;
    return xn;
}

Eigen::MatrixXd Bicycle_model::jacobian_f_x(const Eigen::VectorXd& x,
                                            const Eigen::VectorXd& u) const
{
    double theta = x(2);
    double v     = x(3);
    double delta = u(1);

    double c = std::cos(theta);
    double s = std::sin(theta);
    double t = std::tan(delta);

    Eigen::MatrixXd fx = Eigen::MatrixXd::Zero(4, 4);

    fx(0, 0) = 1.0;
    fx(0, 2) = -v * s * dt_;
    fx(0, 3) =  c * dt_;

    fx(1, 1) = 1.0;
    fx(1, 2) =  v * c * dt_;
    fx(1, 3) =  s * dt_;

    fx(2, 2) = 1.0;
    fx(2, 3) = t / L_ * dt_;

    fx(3, 3) = 1.0;

    return fx;
}


Eigen::MatrixXd Bicycle_model::jacobian_f_u(const Eigen::VectorXd& x,
                                            const Eigen::VectorXd& u) const{
    double v     = x(3);
    double delta = u(1);

    double cos_d = std::cos(delta);
    double sec2  = 1.0 / (cos_d * cos_d);

    Eigen::MatrixXd fu = Eigen::MatrixXd::Zero(4, 2);

    fu(2, 1) = v / L_ * sec2 * dt_;
    fu(3, 0) = dt_;

    return fu;
}

//输入：x0:+U（N步）；输出X（N+1步）
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Bicycle_model::get_init_traj(int N, const Car_state& cur_state, const Control& cur_control)const{
    Eigen::MatrixXd N_state(N+1, 4);
    Eigen::MatrixXd N_control(N, 2);

    for (int i = 0; i < N; ++i) {
        N_control.row(i) << cur_control.a, cur_control.delta;//暂定输入控制量不变
    }
    
    Car_state current = cur_state;//copy
    N_state.row(0) << current.x, current.y, current.theta, current.v;
    for(int k = 0; k < N; ++k){
        Car_state next;
        next.x = current.x + current.v * cos(current.theta) * dt_;
        next.y = current.y + current.v * sin(current.theta) * dt_;
        next.theta = current.theta + (current.v / L_) * tan(cur_control.delta) * dt_;
        next.v = current.v + cur_control.a * dt_;
        N_state.row(k+1)<<next.x, next.y, next.theta, next.v;
        current=next;
    }
    return {N_state, N_control};  
}


}