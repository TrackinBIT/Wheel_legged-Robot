function Uk= MPC_Controller(X0, X_ref_single, I_ext, gama_all, rb_all)
    % MPC_Controller - 基于当前状态和目标状态计算最优地面反作用力
    % rb_all:  [8x1] 列向量 [rb1x; rb1z; rb2x; rb2z; rb3x; rb3z; rb4x; rb4z]
    % gama_all:[4x1] 列向量 [gama1; gama2; gama3; gama4]

    %% ---------- 系统参数 ----------
    mb=60+9.6; mh=3.5; mk=1.5; mw=2.4;
    Lb=0.850; Lw=0.350; Lh=0.200;
    L1=0.300; L2=0.290; r=0.100; 
    g = 9.81;

    %% ---------- 初始化姿态 ----------
    theta = gama_all(1);

    %% ---------- 拆解输入 ----------
    rb1 = rb_all(1:2);
    rb2 = rb_all(3:4);
    rb3 = rb_all(5:6);
    rb4 = rb_all(7:8);

    gama1 = gama_all(1);
    gama2 = gama_all(2);
    gama3 = gama_all(3);
    gama4 = gama_all(4);

    rw1 = rb1 - [0; 0.1];
    rw2 = rb2 - [0; 0.1];
    rw3 = rb3 - [0; 0.1];
    rw4 = rb4 - [0; 0.1];

    %% ---------- 矩阵定义 ----------
    O2 = zeros(2); I2 = eye(2);
    R_theta = [cos(theta) 0; 0 1];
    Cross = [0 -1; 1 0];
    Iyy = (1/12) * mb * (Lb^2 + Lh^2);

    Rg1 = [cos(gama1) sin(gama1); -sin(gama1) cos(gama1)];
    Rg2 = [cos(gama2) sin(gama2); -sin(gama2) cos(gama2)];
    Rg3 = [cos(gama3) sin(gama3); -sin(gama3) cos(gama3)];
    Rg4 = [cos(gama4) sin(gama4); -sin(gama4) cos(gama4)];

    %% ---------- 状态空间模型 ----------
    nx = 8; nu = 8; Np = 20; Nc = 20;
    dt  = 0.1;

    A = [O2 O2 R_theta' O2;
         O2 O2 O2       I2;
         O2 O2 O2       O2;
         O2 O2 O2       O2];

    H = zeros(nx, nu);
    H(6,:)   = [1/Iyy*(Cross*Rg1*rb1)', 1/Iyy*(Cross*Rg2*rb2)', ...
                1/Iyy*(Cross*Rg3*rb3)', 1/Iyy*(Cross*Rg4*rb4)'];
    H(7:8,:) = [I2*Rg1/mb, I2*Rg2/mb, I2*Rg3/mb, I2*Rg4/mb];

    Hw = zeros(nx, nu);
    Hw(6,:)   = [1/Iyy*(Cross*Rg1*rw1)', 1/Iyy*(Cross*Rg2*rw2)', ...
                 1/Iyy*(Cross*Rg3*rw3)', 1/Iyy*(Cross*Rg4*rw4)'];
    Hw(7:8,:) = [I2*Rg1/mb, I2*Rg2/mb, I2*Rg3/mb, I2*Rg4/mb];

    G = [0;0;0;0;0;0;0;-g];

    % 离散化
    Ad  = eye(nx) + dt * A;
    Hd  = dt * H;
    Hwd = dt * Hw;
    Gd  = dt * G;

    % 权重矩阵
    Q_single = diag([1, 1, 1, 20, 1, 1, 1, 0.5]); 
    R_single = 1e-4*eye(nu);
    Q = kron(eye(Np), Q_single);
    R = kron(eye(Nc), R_single);

    % 目标轨迹序列
    X_ref = repmat(X_ref_single, Np, 1);

    %% ---------- 预测矩阵构建 ----------
    A_qp = zeros(nx*Np, nx);
    H_qp = zeros(nx*Np, nu*Nc);
    D_qp = zeros(nx*Np,1);
    for i = 1:Np
        A_qp((i-1)*nx+1:i*nx, :) = Ad^i;
        for j = 1:min(i,Nc)
            H_qp((i-1)*nx+1:i*nx, (j-1)*nu+1:j*nu) = Ad^(i-j)*Hd;
        end
        tempD = zeros(nx,1);
        for j = 0:i-1
            tempD = tempD + Ad^j * (Hwd * I_ext + Gd);
        end
        D_qp((i-1)*nx+1:i*nx) = tempD;
    end

    %% ---------- 构建QP问题 ----------
    H_qp_total = H_qp' * Q * H_qp + R;
    H_qp_total = 0.5 * (H_qp_total + H_qp_total');  % 强制对称
    f_qp = H_qp' * Q * (A_qp*X0 + D_qp - X_ref);

    coder.extrinsic('quadprog');
    U_opt = zeros(nu*Nc,1);
    U_opt = quadprog(H_qp_total, f_qp, [], [], [], [], [], [], [], []);
    Uk = U_opt(1:nu);
end
