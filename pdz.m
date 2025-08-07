function pdz = pdz(Zw_all)
% 输入:
%   Zw_all : [4x1] 当前轮子的 rbz
% 输出:
%   pdz    : 期望质心位置
% 可优化
    z_min = min(Zw_all);

    pdz = 0.28 + z_min;
end
