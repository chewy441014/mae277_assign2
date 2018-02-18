function [tau_id, kappa_id] = system_id()
fresh_run = 0;

if ( fresh_run == 1 )
    % Read r, u, y
    step = read_data('step', '../step.csv', {'t', 'r', 'y', 'u'});
    sine = read_data('sine', '../sine.csv', {'t', 'r', 'y', 'u'});
    ramp = read_data('ramp', '../ramp.csv', {'t', 'r', 'y', 'u'});
    chirp = read_data('chirp', '../chirp.csv', {'t', 'r', 'y', 'u'});

    exps0 = {step, sine, ramp, chirp};

    exps = struct();
    exps.keys = {'step', 'sine', 'ramp', 'chirp'};
    for idx = 1:length(exps.keys)
        exps = setfield(exps, exps.keys{idx}, exps0{idx});
    end


    % Settings
    Ts = 0.001;
    Hz = 1/Ts;

    for idx = 1:length(exps.keys)
        exp = getfield(exps, exps.keys{idx});
        % ========== START Using system ID toolbox
        % Create iddata type
        exp.data_yr = iddata(exp.y, exp.u, Ts);

        % Estimate transfer functions of order np
        np = 2; % position: 2 / velocity: 1
        nz = 0;
        exp.sys = tfest(exp.data_yr, np, nz);

        sys_num_low = exp.sys.num/exp.sys.den(2);
        sys_den_low = [exp.sys.den(1)/exp.sys.den(2) 1 0];
        exp.sys_low = tf(sys_num_low, sys_den_low)
        exp.K = sys_num_low(1);
        exp.tau = sys_den_low(1);
        exps = setfield(exps, exps.keys{idx}, exp);
        clear exp;
    end
    save('id_exp_result.mat', 'exps');
else
    load id_exp_result.mat
end

for  idx = 1:length(exps.keys)
    exp = getfield(exps, exps.keys{idx});
    disp(['==== Original Transfer Function when using ', exp.name ,' signal====']);
    exp.sys
    disp(['==== Trimmed Transfer Function when using ', exp.name ,' signal====']);
    exp.sys_low
end

    tau_id = exps.chirp.tau
    kappa_id = exps.chirp.K
end