function [K,L,sys,Lz] = dcontrold_dir(sys_c)
    % Takes the continuous time LTI open loop plant and applies 
    % state estimator feedback control. 
    
    % Closed loop system is discretized for:
    T = [1/10,1/100,1/1000]; 
    
    % Estimator and State Feedback gains are decided by 
    % discrete time plant analysis
    Kp = [0.8+0.15i, 0.8-0.15i];
    Lp = Kp/10;
    
    % Pole placement is performed for desired discrete time 
    % poles
    n = length(T);
    sys = cell(1,n);
    K = cell(1,n);
    L = cell(1,n);
    Lz = cell(1,n);
    for i = 1:n
        sys_d = c2d(sys_c,T(i));
        A = sys_d.A;
        B = sys_d.B;
        C = sys_d.C;
        D = sys_d.D;
        K{i} = place(A,B,Kp);
        L{i} = place(A',C',Lp)';
        Lz{i} = ss(A-L{i}*C-B*K{i}, L{i}, K{i}, 0, T(i))*ss(A, B, C, 0, T(i));
        s = size(A,1);
        N = 1/(C*inv(eye(s)-A+B*K{i})*B);
        Ao = [A-B*K{i}, B*K{i}; zeros(size(A)), A-L{i}*C];
        Bo = [B*N; zeros(size(B))];
        Co = [C, zeros((size(C)))];
        sys{i} = ss(Ao, Bo, Co, 0, T(i));
    end    
    % Return the observer and state feedback gains and the 
    % new system model for analysis
end