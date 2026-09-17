#include "generate_traj.hpp"
#include "type.hpp"
#include <cmath>

namespace ilqr {


InitTraj::InitTraj(const std::vector<TrajPoint>& control_points)
    : control_points_(control_points) {}

//输入控制点以实现曲线
std::vector<TrajPoint> InitTraj::generate_init_traj(int num_output_points)const{
    if(control_points_.size()<4)
        return control_points_; 

    //控制点转化为矩阵
    int n=static_cast<int>(control_points_.size());
    Eigen::MatrixXd points(2, n);
    for(int i=0;i<n;i++){
        points(0,i)=control_points_[i].x;
        points(1,i)=control_points_[i].y;
    }
    //样条拟合
    typedef Eigen::Spline<double, 2> Spline2d; 
    Spline2d spline = Eigen::SplineFitting<Spline2d>::Interpolate(points,3,Eigen::VectorXd::LinSpaced(n,0, 1));
    std::vector<TrajPoint> output;
    output.reserve(num_output_points);
    for (int i = 0; i < num_output_points; ++i) {
        double t = static_cast<double>(i) / (num_output_points - 1);
        Eigen::VectorXd pt = spline(t); // 返回 (x, y)
        output.push_back({pt(0), pt(1)});
    }
    return output; 
}
} // namespace ilqr 