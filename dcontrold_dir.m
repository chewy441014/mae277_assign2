function [K,L,sys,Lz] = dcontrold_dir(sys_c, Kp, Lp, Ts)
    % Takes the continuous time LTI open loop plant and applies 
    % state estimator feedback control. 
    
    % Closed loop system is discretized for: Ts
    % T = [1/10,1/100,1/1000]; 
    
    % Estimator and State Feedback gains are decided by 
    % discrete time plant analysis
    % Kp = [0.8+0.15i, 0.8-0.15i];
    % Lp = Kp/10;
    
    % Pole placement is performed for desired discrete time poles
    sys_d = c2d(sys_c, Ts);
    A = sys_d.A;
    B = sys_d.B;
    C = sys_d.C;
    D = sys_d.D;
    
    % K : Feedback Gain
    K = place(A,B,Kp);
    
    % L : Observer Gain
    L = place(A',C',Lp)';
    
    % Lz: Loop Gain
    Lz = ss(A - L*C - B*K, L, K, 0, Ts) * ss(A, B, C, 0, Ts);
    
    % sys: Closed Loop System
    nState = size(A,1);
    N = 1/(C * inv(eye(nState) - A+B*K) * B);
    Ao = [A - B*K, B*K; zeros(size(A)), A - L*C];
    Bo = [B*N; zeros(size(B))];
    Co = [C, zeros((size(C)))];
    sys = ss(Ao, Bo, Co, 0, Ts);

    % Return the observer and state feedback gains and the 
    % new system model for analysis
end