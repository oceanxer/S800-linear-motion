# S800-linear-motion
基于大连探索海洋科技有限公司S800水下机器人二次开发，硬件设备为ROV平台
# scripts
## controller_lib.py
控制器库文件，内置了单自由度PID控制器类型
## oceanxer_lib.py
水下机器人控制py文件，包括机械臂Jacobian的计算，S800 ROV推力分配及转化为PWM信号方法
## param.yaml
机器人参数文件，包括推力分配系数矩阵，以及由推进器PWM2Thrust曲线得到的推力转PWM配置参数
## linear_motion.py
机器人运动控制核心文件，其功能是连接机器人，配置深度、Roll以及Pitch控制器，并控制机器人的实施深度，具体见S800_ROV类
# run

`git clone https://github.com/oceanxer/S800-linear-motion.git`

`python linear_motion.py`