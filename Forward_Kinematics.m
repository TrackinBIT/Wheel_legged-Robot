function [qh, qk] = Forward_Kinematics(rb, legIndex)
% 输入
% legIndex ：轮腿编号
% rb = [rbx; rbz]，腿轮相对质心的位置向量
% 输出
% qh, qk ：髋关节和膝关节角度（单位：弧度）
    
    % 参数设置
    L  = 0.850 - 0.05 * 2;  % 车体总长减去边缘
    L1 = 0.300;  % 大腿长度
    L2 = 0.290;  % 小腿长度

    % 解析输入
    x = rb(1);
    z = rb(2);
    
    % 计算 dx
    if ismember(legIndex, [1, 2])  % 前腿
        dx = x - L/2;
    else                           % 后腿
        dx = -x - L/2;
    end
    dz = z;
    r = sqrt(dx^2 + dz^2);  % 髋到轮心的距离

    % 计算膝关节角度 qk
    cos_qk = (r^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_qk = max(min(cos_qk, 1), -1); 

    % 计算角度
    if ismember(legIndex, [1, 2])  % 前腿：膝关节向后弯
        qk = -acos(cos_qk);
        phi = atan2(-dz, dx);
        cos_psi = (L1^2 + r^2 - L2^2) / (2 * L1 * r);
        cos_psi = max(min(cos_psi, 1), -1);
        psi = acos(cos_psi);
        qh = phi + psi;
    else                           % 后腿：膝关节向前弯
        qk = acos(cos_qk);
        phi = atan2(-dz, dx);
        cos_psi = (L1^2 + r^2 - L2^2) / (2 * L1 * r);
        cos_psi = max(min(cos_psi, 1), -1);
        psi = acos(cos_psi);
        qh = pi - (phi + psi);
    end
end
