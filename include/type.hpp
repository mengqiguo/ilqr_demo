#pragma once
#include <Eigen/Dense>
#include <limits>
#include <vector>

namespace ilqr {

struct Car_state{
    double x;
    double y;
    double theta;//航向角
    double v;
};
struct Control{
    double a;
    double delta;//方向盘转角
};

struct TrajPoint{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
    double v = 0.0;

    Eigen::VectorXd toEigen() const {
        Eigen::VectorXd state(4);
        state << x, y, theta, v;
        return state;
    }
};

//TODO 结构体多且重复，考虑合并或优化
struct StageCostDerivs {
    Eigen::VectorXd l_x;    // ∂l/∂x    (n×1)
    Eigen::VectorXd l_u;    // ∂l/∂u    (m×1)
    Eigen::MatrixXd l_xx;   // ∂²l/∂x²   (n×n)
    Eigen::MatrixXd l_uu;   // ∂²l/∂u²   (m×m)
    Eigen::MatrixXd l_ux;   // ∂²l/∂u∂x  (m×n)
};

struct TerminalCostDerivs {
    Eigen::VectorXd l_f_x;   // ∂l_f/∂x   (n×1)
    Eigen::MatrixXd l_f_xx;  // ∂²l_f/∂x² (n×n)
};

struct DerivResult {
    std::vector<Eigen::MatrixXd> A;      // 长度 N, 每个 n×n
    std::vector<Eigen::MatrixXd> B;      // 长度 N, 每个 n×m
    std::vector<Eigen::VectorXd> l_x;    // 长度 N, 每个 n
    std::vector<Eigen::VectorXd> l_u;    // 长度 N, 每个 m
    std::vector<Eigen::MatrixXd> l_xx;   // 长度 N, 每个 n×n
    std::vector<Eigen::MatrixXd> l_uu;   // 长度 N, 每个 m×m
    std::vector<Eigen::MatrixXd> l_ux;   // 长度 N, 每个 m×n
    Eigen::VectorXd              l_f_x;  // n
    Eigen::MatrixXd              l_f_xx; // n×n
};

struct BackwardResult {
    bool success = false;
    std::vector<Eigen::MatrixXd> K;  
    std::vector<Eigen::VectorXd> k; 
};
struct ForwardResult {
    Eigen::MatrixXd X;   // (N+1)×nx
    Eigen::MatrixXd U;   // N×nu
};

struct ILQRResult {
    bool            success = false;
    int             iter    = 0;
    double          cost    = std::numeric_limits<double>::infinity();
    Eigen::MatrixXd X;   // (N+1)×nx
    Eigen::MatrixXd U;   // N×nu
};

//TODO ：尽量用结构体  函数返回尽量不用{}/或者直接传给输入的空值
} // namespace ilqr