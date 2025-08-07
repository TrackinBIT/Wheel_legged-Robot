function vx = vx(t)
    v_max = 0.7;  % 目标速度
    T = 1;        % 上升时间长度（秒）

    if t <= 0
        vx = 0;
    elseif t < T
        s = t / T;
        vx = v_max * (3*s^2 - 2*s^3);  
    else
        vx = v_max;
    end
end
