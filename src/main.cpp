#include "bicycle_model.hpp"
#include "type.hpp"
#include "generate_traj.hpp"
#include "ilqr_solver.hpp"
#include "plot.cpp"
#include <yaml-cpp/yaml.h>

#include <iostream>

using namespace ilqr;

//实现参考轨迹的平滑处理
int main() {
    std::string config_path = "config/config.yaml";
    YAML::Node config;
    try {
        config = YAML::LoadFile(config_path);
    } catch (const YAML::Exception& e) {
        std::cerr << "Failed to load config.yaml: " << e.what() << std::endl;
        return -1;
    }

    double L = config["vehicle"]["L"].as<double>();
    double dt = config["vehicle"]["dt"].as<double>();
    Bicycle_model model(L, dt);
    YAML::Node ilqr_node = config["ilqr"];
    ILQRSolver ilqr_solver(model, ilqr_node);
    const std::vector<double> state_values = config["ego_init_state"].as<std::vector<double>>();
    const std::vector<double> control_values = config["ego_init_u"].as<std::vector<double>>();
    if (state_values.size() != 4 || control_values.size() != 2) {
        std::cerr << "Invalid initial state or control size" << std::endl;
        return -1;
    }
    Car_state cur_state{state_values[0], state_values[1], state_values[2], state_values[3]};
    Control cur_u{control_values[0], control_values[1]};

    std::vector<TrajPoint> control_points;
    for (const auto& waypoint : config["waypoints"]) {
        control_points.push_back({waypoint[0].as<double>(), waypoint[1].as<double>()});
    }
    InitTraj trajectory_generator(control_points);
    std::vector<TrajPoint> smooth_traj = trajectory_generator.generate_init_traj(ilqr_node["N"].as<int>() + 1);

    ILQRResult result = ilqr_solver.solve(ilqr_node, cur_state, cur_u, smooth_traj);

}