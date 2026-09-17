#include <ilqr_solver.hpp>
#include <bicycle_model.hpp>
#include <iostream>
#include <Eigen/Dense>
#include <type.hpp>


namespace ilqr{

//构造函数
ILQRSolver::ILQRSolver(const Bicycle_model& model,const YAML::Node& ilqr_node)
    : model_(model), ilqr_node_(ilqr_node) {}


//输入：x_ref+x0+u_init+config；输出：x_opt+u_opt

ILQRResult ILQRSolver::solve(const YAML::Node& ilqr_node,const Car_state& cur_state, const Control& cur_control, const std::vector<TrajPoint>& ref_trajectory) {
    //TODO  参考轨迹未定
    ILQRResult result;
    int N=ilqr_node["N"].as<int>();
    auto [N_state, N_control] = model_.get_init_traj(N, cur_state, cur_control);
    double J=compute_cost(N_state, N_control, ref_trajectory, ilqr_node);
    
    int max_iter=ilqr_node["max_iter"].as<int>();
    double init_lamb=ilqr_node["init_lamb"].as<double>();
    double max_lamb=ilqr_node["max_lamb"].as<double>();
    double eps_cost = ilqr_node["eps_cost"].as<double>();

    for(int i=0;i<max_iter;i++){
        DerivResult derivs = compute_derivatives(N_state, N_control, ref_trajectory, ilqr_node);
        bool accepted = false;
        while (!accepted) {
            BackwardResult backward_result = backward_pass(N_state, N_control, derivs, init_lamb);
            if (!backward_result.success) {
                init_lamb *= 10;//TODO： mu数值没有设定明白
                if (init_lamb > max_lamb) {
                    result.X = N_state;
                    result.U = N_control;
                    result.cost = J;
                    result.iter = i;
                    return result;      // 失败退出
                }
                continue;
            }
        //回溯线搜索
            bool improved = false;
            double J_old = J;
            for (double alpha = 1.0; alpha > 1e-6; alpha *= 0.5){
                // X_new, U_new = forward(X, U,backward_result.K,backward_result.k,x0,alpha)
                ForwardResult forward_result = forward_pass(N_state, N_control, backward_result.K, backward_result.k, alpha);
                //TODO:迭代的参考轨迹是不变的吧
                double J_new = compute_cost(forward_result.X, forward_result.U, ref_trajectory, ilqr_node);
                if (J_new < J) {   // 可换成 Armijo 条件
                    N_state = forward_result.X;
                    N_control = forward_result.U;
                    J = J_new;
                    init_lamb = std::max(init_lamb / 10.0, 1e-6);
                    accepted = true;
                    improved  = true;
                    break;
                } 
                if (!improved) {
                    init_lamb *= 10;
                    if (init_lamb > max_lamb) {
                        result.X = N_state;
                        result.U = N_control;
                        result.cost = J;
                        result.iter = i;
                        return result;      // 失败退出
                    }
                }

            }

            if (std::abs(J - J_old) < eps_cost * (1.0 + std::abs(J_old))) {
                break;
            }
        }
    }
    result.success = true;
    result.iter    = max_iter;      // 或收敛处的 iter
    result.cost    = J;
    result.X       = N_state;
    result.U       = N_control;
    return result;

}

//ilqr后向传播过程
//输入：derivs+mu；输出：K[k]反馈增益+k[k]前馈增益
BackwardResult ILQRSolver::backward_pass(const Eigen::MatrixXd& N_state, const Eigen::MatrixXd& N_control, const DerivResult& derivs, double init_lamb) {
    int N = static_cast<int>(N_control.rows());          // 时间步数
    int nx = N_state.cols();                            // 状态维度
    int nu = N_control.cols();                        // 控制维度
    
    BackwardResult result;

    //终端代价的导数
    DerivResult derivs_ = derivs;
    Eigen::VectorXd V_x = derivs_.l_f_x;
    Eigen::MatrixXd V_xx = derivs_.l_f_xx;

    //倒序循环 (k = N-1 ... 0)
    for (int k = N - 1; k >= 0; --k) {
        Eigen::VectorXd l_x = derivs_.l_x[k];
        Eigen::VectorXd l_u = derivs_.l_u[k];
        Eigen::MatrixXd l_xx = derivs_.l_xx[k];
        Eigen::MatrixXd l_uu = derivs_.l_uu[k];
        Eigen::MatrixXd l_ux = derivs_.l_ux[k];   // 通常为 nu x nx

        Eigen::MatrixXd f_x = derivs_.A[k];
        Eigen::MatrixXd f_u = derivs_.B[k];  // nx x nu

        Eigen::VectorXd Q_x = l_x + f_x.transpose() * V_x;
        Eigen::VectorXd Q_u = l_u + f_u.transpose() * V_x;

        Eigen::MatrixXd Q_xx = l_xx + f_x.transpose() * V_xx * f_x;
        Eigen::MatrixXd Q_uu = l_uu + f_u.transpose() * V_xx * f_u;
        Eigen::MatrixXd Q_ux = l_ux + f_u.transpose() * V_xx * f_x;   // nu x nx

        //正则化
        Q_uu += init_lamb * Eigen::MatrixXd::Identity(nu, nu);
        Eigen::LDLT<Eigen::MatrixXd> ldlt(Q_uu);
        if (ldlt.info() != Eigen::Success) {
            result.success = false;
            return result;
        }
        Eigen::MatrixXd Q_uu_inv = Q_uu.inverse();
        Eigen::MatrixXd K = -Q_uu_inv * Q_ux;    // nu x nx
        Eigen::VectorXd d = -Q_uu_inv * Q_u;     // nu x 1

        V_x = Q_x - K.transpose() * Q_uu * d;//更新
        V_xx = Q_xx - K.transpose() * Q_uu * K;

        result.K.push_back(K);
        result.k.push_back(d);
        result.success = true;
    }
    //TODO:什么情况需要返回false
    return result;
}

//ilqr前向传播过程
//输入：X_old+U_old+K+K+x0+alpha；输出：X_new+U_new
ForwardResult ILQRSolver::forward_pass(const Eigen::MatrixXd& N_state,
                              const Eigen::MatrixXd& N_control,
                              const std::vector<Eigen::MatrixXd>& K_list_,
                              const std::vector<Eigen::VectorXd>& d_list_,
                              double alpha) {

    int N = static_cast<int>(N_control.rows());   // 控制步数
    ForwardResult out;
    out.X.resize(N + 1, N_state.cols());
    out.U.resize(N, N_control.cols());
    out.X.row(0) = N_state.row(0);//保留初始状态
    //正向传播
    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd x_new = out.X.row(k).transpose();
        const Eigen::VectorXd& x_old = N_state.row(k).transpose();
        const Eigen::VectorXd& u_old = N_control.row(k).transpose();
        const Eigen::VectorXd dx = x_new - x_old;
        const Eigen::VectorXd du = alpha * d_list_[k] + K_list_[k] * dx;
        out.U.row(k) = (u_old + du).transpose();
        out.X.row(k + 1) = model_.dynamics(x_new, u_old + du).transpose();
    }
    return out;
}
//代价计算
//TODO：是否带入阶段+终端函数
double ILQRSolver::compute_cost(
        const Eigen::MatrixXd&                       N_state,         
        const Eigen::MatrixXd&                       N_control, 
        const std::vector<TrajPoint>&                ref_trajectory,    
        const YAML::Node&                            ilqr_node      
        ) 
{
    //TODO:  需要设置标志定义冷启动热启动
    //TODO ：计算最近参考点待检查

    int N = ilqr_node["N"].as<int>();
    auto Q_vec = ilqr_node["Q"].as<std::vector<double>>();
    auto R_vec = ilqr_node["R"].as<std::vector<double>>();
    int n = static_cast<int>(N_state.cols());
    int m = static_cast<int>(N_control.cols());

    //向量转对角矩阵
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < std::min<int>(n, Q_vec.size()); ++i) Q(i, i) = Q_vec[i];
    for (int i = 0; i < std::min<int>(m, R_vec.size()); ++i) R(i, i) = R_vec[i];
    Eigen::MatrixXd Qf = Q;//终端与其一致

    std::vector<Eigen::VectorXd> x_ref(N + 1);
    for (int k = 0; k <= N; ++k) {
        x_ref[k] = Eigen::VectorXd(4);
        x_ref[k] << ref_trajectory[k].x, ref_trajectory[k].y, ref_trajectory[k].theta, ref_trajectory[k].v;
    }
    
    double cost = 0.0;
    for (int k = 0; k < N; ++k) {
        Eigen::VectorXd x = N_state.row(k).transpose();
        Eigen::VectorXd u = N_control.row(k).transpose();
        Eigen::VectorXd dx = x - x_ref[k];
        Eigen::VectorXd du = u;

        cost += dx.transpose() * Q * dx;
        cost += du.transpose() * R * du;
    }
    //终端代价
        Eigen::VectorXd xN = N_state.row(N).transpose();
        Eigen::VectorXd dxN = xN - x_ref[N];
        cost += dxN.transpose() * Qf * dxN;
    return cost;
}

//阶段代价
StageCostDerivs ILQRSolver::stage_cost_derivatives(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& u,
        const Eigen::VectorXd& x_ref,
        const Eigen::MatrixXd& Q,
        const Eigen::MatrixXd& R){
    StageCostDerivs d;
    Eigen::VectorXd dx = x - x_ref;

    d.l_x = 2 * Q * dx;
    d.l_u = 2 * R * u;
    d.l_xx = 2 * Q;
    d.l_uu = 2 * R;
    d.l_ux = Eigen::MatrixXd::Zero(u.size(), x.size());  // 无交叉项
    return d;
}

//终端代价
TerminalCostDerivs ILQRSolver::terminal_cost_derivatives(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& x_ref,
        const Eigen::MatrixXd& Qf)
{
    TerminalCostDerivs d;
    Eigen::VectorXd dx = x - x_ref;

    d.l_f_x  = 2.0 * Qf * dx;   // n
    d.l_f_xx = 2.0 * Qf;        // n×n
    return d;
}
//计算最近参考点
Eigen::MatrixXd ILQRSolver::compute_closest_ref_point(const std::vector<TrajPoint>& ref_trajectory, const Eigen::MatrixXd& N_state) {
    //计算距离每行N——state最近的参考点，组成新的参考轨迹
    Eigen::MatrixXd closest_ref_traj(N_state.rows(), 4);
    for (int i = 0; i < N_state.rows(); ++i) {
        double min_dist = std::numeric_limits<double>::max();
        int closest_idx = 0;
        for (int j = 0; j < ref_trajectory.size(); ++j) {
            double dx = N_state(i, 0) - ref_trajectory[j].x;
            double dy = N_state(i, 1) - ref_trajectory[j].y;
            double dist = dx * dx + dy * dy;
            if (dist < min_dist) {
                min_dist = dist;
                closest_idx = j;
            }

        }
        closest_ref_traj.row(i) << ref_trajectory[closest_idx].x, ref_trajectory[closest_idx].y,
                                   ref_trajectory[closest_idx].theta, ref_trajectory[closest_idx].v;
    }
    return closest_ref_traj; //返回最近参考点组成的参考轨迹
}



//求解器求导:求动力学雅可比和代价导数
//输入：X+U+ x_ref+params；输出：A[k]/B[k]/l_x[k]/l_u[k]/l_xx[k]/l_uu[k]/l_ux[k]/l_f_x/l_f_xx
DerivResult ILQRSolver::compute_derivatives(const Eigen::MatrixXd& N_state, const Eigen::MatrixXd& N_control, const std::vector<TrajPoint>& ref_trajectory, const YAML::Node& ilqr_node){

    int N = static_cast<int>(N_control.rows());
    int n = static_cast<int>(N_state.cols());
    int m = static_cast<int>(N_control.cols());

    auto Q_vec = ilqr_node["Q"].as<std::vector<double>>();
    auto R_vec = ilqr_node["R"].as<std::vector<double>>();
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n, n);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(m, m);
    for (int i = 0; i < std::min<int>(n, Q_vec.size()); ++i) Q(i, i) = Q_vec[i];
    for (int i = 0; i < std::min<int>(m, R_vec.size()); ++i) R(i, i) = R_vec[i];
    Eigen::MatrixXd Qf = Q; 

    std::vector<Eigen::VectorXd> x_ref(N + 1);
    for (int k = 0; k <= N; ++k) {
        x_ref[k] = ref_trajectory[k].toEigen();
    }

    //预分配结果
    DerivResult res;
    res.A   .resize(N);
    res.B   .resize(N);
    res.l_x .resize(N);
    res.l_u .resize(N);
    res.l_xx.resize(N);
    res.l_uu.resize(N);
    res.l_ux.resize(N);

    for (int k = 0; k < N; ++k) {
        const Eigen::VectorXd x = N_state  .row(k).transpose();
        const Eigen::VectorXd u = N_control.row(k).transpose();

        // 动力学雅可比
        res.A[k] = model_.jacobian_f_x(x, u);   // n×n
        res.B[k] = model_.jacobian_f_u(x, u);   // n×m

        // 阶段代价导数
        StageCostDerivs d = stage_cost_derivatives(x, u, x_ref[k], Q, R);
        res.l_x [k] = d.l_x;
        res.l_u [k] = d.l_u;
        res.l_xx[k] = d.l_xx;
        res.l_uu[k] = d.l_uu;
        res.l_ux[k] = d.l_ux;
    }

    Eigen::VectorXd xN = N_state.row(N).transpose();
    TerminalCostDerivs df = terminal_cost_derivatives(xN, x_ref[N], Qf);
    res.l_f_x  = df.l_f_x;
    res.l_f_xx = df.l_f_xx;
    return res;
}

}



