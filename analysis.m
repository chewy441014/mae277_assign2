function analysis(sysModel, sysCL, K, L, Ts)
    %% ============================== %%
    % :param sysModel: System data
    % :param sysCL: Closed loop model
    % :param K: Controller gain
    % :param L: Observer gain
    % :param Ts: Frequency in Hz

    [Ac, Bc, Cc, Dc] = ssdata(sysModel);
    nStates = size(Ac, 1);
    nInputs = size(Bc, 2);
    nOutputs = size(Cc, 1);
    N = 1 / (Cc * inv(-sysCL.A) * Bc);

    %% ============================== %%
    %% Closed-loop Plant Frequency Response
    sysCL_Gc = sysCL * N;
    figure(Ts*10+1);
    step(sysCL_Gc); title('Closed-Loop Plant Frequency Response');

    %% ============================== %%
    %% Time-Domain Analysis
    figure(Ts*10+2);
    step(sysCL_Gc); title('Step Response');
    stepInfo = stepinfo(sysCL_Gc);

    %% ============================== %%
    %% Frequency-Domain Analysis %%
    % Controllers
    ctrlFB = minreal(-tf(ss(Ac - L*Cc - Bc*K, L, K, zeros(nInputs, nOutputs))));

    % Loop Gain
    loopGain = minreal(-series(-ctrlFB, sysModel));

    % Stability Margins
    figure(Ts*10+3);
    bode(loopGain); title([sysCL.Name, ' Loop Gain']);

    figure(Ts*10+4);
    nyquist(loopGain); title([sysCL.Name, ' Nyquist']);

    % Sensitivity
    S = 1/(1 + loopGain);
    S_peak = getPeakGain(S);
    VGM = S_peak / (S_peak - 1);

    figure(Ts*10+5);
    bodemag(S); title([sysCL.Name, ' Sensitivity']);

    % Complementary Sensitivity
    T = feedback(loopGain, 1);

    figure(Ts*10+6);
    bodemag(T); title([sysCL.Name, ' Complementary Sensitivity']);
end