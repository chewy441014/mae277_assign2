function analysis(idt, sysModel, sysCL, K, L, Hz)
    %% ============================== %%
    % :param idt: Indicator value for smart plotting
    % :param sysModel: System data
    % :param sysCL: Closed loop model
    % :param K: Controller gain
    % :param L: Observer gain
    % :param Hz: Frequency in Hz
    
    [Ac, Bc, Cc, Dc] = ssdata(sysModel);
    nStates = size(Ac, 1);
    nInputs = size(Bc, 2);
    nOutputs = size(Cc, 1);
    
    if nargin < 6
        Hz = 0;
        N = 1 / (Cc * inv(-sysCL.A) * Bc);
    else
        sysModel = c2d(sysModel, 1/Hz);
        [Ac, Bc, Cc, Dc] = ssdata(sysModel);
        N = 1 / (Cc * inv(eye(nStates)-sysCL.A) * Bc);
        Ts = 1/Hz;
    end
    %% ============================== %%
    %% Closed-loop Plant Frequency Response
    sysCL_Gc = sysCL * N;
    figure(idt*10+1);
    bode(sysCL_Gc); title('Closed-Loop Plant Frequency Response');

    %% ============================== %%
    %% Time-Domain Analysis
    figure(idt*10+2);
    step(sysCL_Gc); title('Step Response');
    stepInfo = stepinfo(sysCL_Gc)

    %% ============================== %%
    %% Frequency-Domain Analysis %%
    % Controllers
    if nargin < 6
        ctrlFB = minreal(-tf(ss(Ac - L*Cc - Bc*K, L, K, zeros(nInputs, nOutputs))));
    else
        ctrlFB = minreal(-tf(ss(Ac - L*Cc - Bc*K, L, K, zeros(nInputs, nOutputs), Ts)));
    end

    % Loop Gain
    loopGain = minreal(series(-ctrlFB, sysModel));
    
    % Analysis
    analysisGivenLoopGain(loopGain, idt);
end