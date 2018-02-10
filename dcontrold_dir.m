function [K,L,sys] = dcontrold_dir(sys_c)
    % Takes the continuous time LTI open loop plant and applies 
    % state estimator feedback control. 
    
    % Estimator and State Feedback gains are decided by 
    % discrete time plant analysis
    Lp = [];
    Kp = [];
    
    % Pole placement is performed for desired discrete time 
    % poles
    
    % Closed loop system is discretized for:
    T = [1/10,1/100,1/1000]; 
    
    % Return the observer and state feedback gains and the 
    % new system model for analysis
end