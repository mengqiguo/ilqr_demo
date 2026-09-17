#include <type.hpp>
#include <bicycle_model.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>



namespace ilqr{


class ILQRSolver {

public:
    //构造函数
    ILQRSolver(const Bicycle_model& model,const YAML::Node& ilqr_node);

    ILQRResult solve(const YAML::Node& ilqr_node,const Car_state& cur_state, const Control& cur_control, const std::vector<TrajPoint>& ref_trajectory);
    ForwardResult forward_pass(
        const Eigen::MatrixXd& X_old,
        const Eigen::MatrixXd& U_old,
        const std::vector<Eigen::MatrixXd>& K_list,
        const std::vector<Eigen::VectorXd>& d_list,
        double alpha);
    BackwardResult backward_pass(const Eigen::MatrixXd& N_state, const Eigen::MatrixXd& N_control, const DerivResult& derivs, double init_lamb);
    double compute_cost(const Eigen::MatrixXd& N_state, const Eigen::MatrixXd& N_control, const std::vector<TrajPoint>& ref_trajectory, const YAML::Node& ilqr_node);
    DerivResult compute_derivatives(const Eigen::MatrixXd& N_state, const Eigen::MatrixXd& N_control, const std::vector<TrajPoint>& ref_trajectory, const YAML::Node& ilqr_node);
    Eigen::MatrixXd compute_closest_ref_point(const std::vector<TrajPoint>& ref_trajectory, const Eigen::MatrixXd& N_state);
    StageCostDerivs stage_cost_derivatives(const Eigen::VectorXd& x, const Eigen::VectorXd& u, const Eigen::VectorXd& x_ref, const Eigen::MatrixXd& Q, const Eigen::MatrixXd& R);
    TerminalCostDerivs terminal_cost_derivatives(const Eigen::VectorXd& x, const Eigen::VectorXd& x_ref, const Eigen::MatrixXd& Qf);
    

private:
//TODO: 输入轨迹+控制--先分再合起来
    //变量
    ilqr::Car_state cur_state;
    ilqr::Control cur_control;
    ilqr::TrajPoint traj_point;
    Bicycle_model model_;
    YAML::Node ilqr_node_;
};
}