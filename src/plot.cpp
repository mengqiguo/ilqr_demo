#include "matplotlibcpp.h"
#include <vector>

namespace plt = matplotlibcpp;

//TODO：要添加修改前与修改后的
void plot_trajectory(const std::vector<ilqr::TrajPoint>& control_points,const std::vector<ilqr::TrajPoint>& smooth_trajectory){
    std::vector<double> control_x, control_y;
    for (const auto& pt : control_points) {
        control_x.push_back(pt.x);
        control_y.push_back(pt.y);
    }

    std::vector<double> smooth_x, smooth_y;
    for (const auto& pt : smooth_trajectory) {
        smooth_x.push_back(pt.x);
        smooth_y.push_back(pt.y);
    }

    plt::figure_size(1200, 780); 
    plt::named_plot("Control Points", control_x, control_y, "ro-");
    plt::named_plot("Smooth Trajectory", smooth_x, smooth_y, "b-");
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::title("Trajectory");
    plt::legend();
    plt::show();
}