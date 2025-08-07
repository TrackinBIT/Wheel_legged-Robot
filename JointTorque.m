function Torque_all = JointTorque( fd_all,rb_all)
% 输入：
%   rb_all : [8×1] 4条腿的 [rbx1; rbz1; ... rbx4; rbz4]
%   fd_all : [8×1] 4条腿的 [fdx1; fdz1; ... fdx4; fdz4]
% 输出：
%   Torque_all : [8×1] 雅克比矩阵映射到每个轮腿的大腿和小腿关节前馈力矩

    L1 = 0.30;
    L2 = 0.29;
    g  = 9.81;
    mw = 2.4;
    torque_max = 144;

    % 初始化
    Torque_all = zeros(8,1);

    for i = 1:4
        idx = (i-1)*2 + 1;
        
        rb = rb_all(idx:idx+1);

        % 反解关节角
        [qh, qk] = Forward_Kinematics(rb, i);

        fd_x = fd_all(idx);
        fd_z = fd_all(idx+1) - mw * g;

        % 雅可比矩阵
        J = [ -L1*sin(qh) - L2*sin(qh + qk),  -L2*sin(qh + qk);
               L1*cos(qh) + L2*cos(qh + qk),   L2*cos(qh + qk) ];

        % 力 → 力矩
        torque_leg = J' * [fd_x; fd_z];

        % 限幅
        torque_leg = max(min(torque_leg, torque_max), -torque_max);

        Torque_all(idx:idx+1) = torque_leg;
    end
end
