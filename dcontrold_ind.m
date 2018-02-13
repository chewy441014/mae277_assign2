function [K,L,sysCL] = dcontrold_ind(sys_c, desFPPoles, desObsPoles)
% :param sys_c: Actual system
% :param desFPPoles: Desired feedback poles
% :param desObsPoles: Desired observer poles

    % Controller is discretized for:
    T = [1/10,1/100,1/1000]; 
    
    % Takes the continuous time LTI open loop plant and applies 
    % state estimator feedback control.
    [Ac, Bc, Cc, Dc] = ssdata(sys_c);
    nStates = size(Ac, 1);
    nInputs = size(Bc, 2);
    nOutputs = size(Cc, 1);
    
    % Estimator and State Feedback gains are decided by 
    % continuous time plant analysis
    K = place(Ac, Bc, desFbPoles);
    L = place(Ac', Cc', desObsPoles).';
    
    % Stabilized system
    A_stb = Ac - Bc*K;
    
    % Feedforward gain
    N = 1 / (Cc * inv(-A_stb) * Bc);
    
    % Return the observer and state feedback gains and the 
    % new system model for analysis
    sysCL = minreal(ss(A_stb, Bc, Cc, zeros(nInputs, nOutputs)));
end