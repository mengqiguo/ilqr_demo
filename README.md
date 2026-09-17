# iLQR Bicycle Model Demo
一个基于自行车运动学模型的 iLQR（Iterative Linear Quadratic Regulator）轨迹优化demo（靠近参考线）
## 功能
- 自行车运动学模型
- B样条+控制点
- 使用iLQR进行轨迹优化
## 环境要求
- CMake >= 3.10
- 支持 C++17 的编译器
- Eigen3 >= 3.3
- yaml-cpp >= 0.8
- Python 3
- Python 开发包
- NumPy 及其 Python 开发头文件
- matplotlib（如果启用绘图功能）
```bash
sudo apt update
sudo apt install -y build-essential cmake libeigen3-dev libyaml-cpp-dev \
    python3-dev python3-numpy python3-matplotlib
```

## 编译与运行

```bash
cd ilqr_demo/bulid
cmake ..
make 
```
```bash
cd ilqr_demo
build/run_bicycle
```

## 目录结构

```text
.
├── CMakeLists.txt             
├── config/
│   └── config.yaml            
├── include/
│   ├── bicycle_model.hpp      
│   ├── generate_traj.hpp      
│   ├── ilqr_solver.hpp        
│   ├── matplotlibcpp.h        
│   └── type.hpp               
├── src/
│   ├── bicycle_model.cpp      # 车辆动力学和雅可比矩阵
│   ├── generate_traj.cpp      # 样条轨迹生成
│   ├── ilqr_solver.cpp        # iLQR 求解流程
│   ├── main.cpp               # 程序入口
│   └── plot.cpp               # 轨迹绘图函数
└── README.md
```
