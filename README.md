# Wheel_legged-Robot
基于MPC的轮式四足机器人动态运动控制仿真

● 仿真环境：MATLAB 2022b / Simscape Multibody
<img width="1711" height="1044" alt="main" src="https://github.com/user-attachments/assets/43f4722d-d7e1-403a-9ec1-a79bea424059" />



● WLV ： Simscape仿真主文件

● MPC_Controller.m ： 基于当前状态和目标状态计算最优地面反作用力

● Torque_all.m ： 地面反作用力->雅克比矩阵->关节力矩

● Forward_Kinematics.m ： 基于轮心位置 -> 关节角度

● GroundSlope.m ： 地面坡度简单估计

项目复现自论文《复杂地形下轮腿复合机动平台动态运动控制》

个人学习制作，欢迎交流

 E-mail ： BIT_track@163.com
