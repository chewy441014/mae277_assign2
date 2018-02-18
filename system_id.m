function [tau_id, kappa_id] = system_id(filepath, Ts)
    % Read r, u, y
    exp = read_data(filepath, {'t', 'r', 'y', 'u'});
    
    % ========== START Using system ID toolbox
    % Create iddata type
    exp.data_yr = iddata(exp.y, exp.u, Ts);

    % Estimate transfer functions of order np
    np = 2; % position: 2 / velocity: 1
    nz = 0;
    exp.sys = tfest(exp.data_yr, np, nz);

    sys_num_low = exp.sys.num/exp.sys.den(2);
    sys_den_low = [exp.sys.den(1)/exp.sys.den(2) 1 0];
    exp.sys_low = tf(sys_num_low, sys_den_low);
    exp.K = sys_num_low(1);
    exp.tau = sys_den_low(1);
    
    tau_id = exp.tau;
    kappa_id = exp.K;
end