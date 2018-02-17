function [K,L,sysCL,N] = dcontrold_ind(sys_c, desFbPoles, desObsPoles, Ts)
% :param sys_c: Actual system
% :param desFPPoles: Desired feedback poles
% :param desObsPoles: Desired observer poles
    
    % Takes the continuous time LTI open loop plant and applies 
    % state estimator feedback control.
    [A, B, C, D] = ssdata(sys_c);
    
    % Estimator and State Feedback gains are decided by 
    % continuous time plant analysis
    K = place(A, B, desFbPoles);
    L = place(A', C', desObsPoles).';
    
    Ao = [A - B*K, B*K; zeros(size(A)), A - L*C];
    Bo = [B; zeros(size(B))];
    Co = [C, zeros((size(C)))];
    sysCL = c2d(minreal(ss(Ao, Bo, Co, 0)),Ts);
    sysCL.Name = 'Indirect Control';
    N = 1 / (C * inv(eye(size(sysCL.A))-sysCL.A) * B);
    
end