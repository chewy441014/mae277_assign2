function analysis(sysModel, sysCL, K, L, Hz)
    %% ============================== %%
    % :param sysModel: System data
    % :param sysCL: Closed loop model
    % :param K: Controller gain
    % :param L: Observer gain
    % :param Hz: Frequency in Hz
    
    [Ac, Bc, Cc, Dc] = ssdata(sysModel);
    nStates = size(Ac, 1);
    nInputs = size(Bc, 2);
    nOutputs = size(Cc, 1);
    
    if nargin < 5
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
    figure(Hz*10+1);
    bode(sysCL_Gc); title('Closed-Loop Plant Frequency Response');

    %% ============================== %%
    %% Time-Domain Analysis
    figure(Hz*10+2);
    step(sysCL_Gc); title('Step Response');
    stepInfo = stepinfo(sysCL_Gc)

    %% ============================== %%
    %% Frequency-Domain Analysis %%
    % Controllers
    if nargin < 5
        ctrlFB = minreal(-tf(ss(Ac - L*Cc - Bc*K, L, K, zeros(nInputs, nOutputs))));
    else
        ctrlFB = minreal(-tf(ss(Ac - L*Cc - Bc*K, L, K, zeros(nInputs, nOutputs), Ts)));
    end

    % Loop Gain
    loopGain = minreal(series(-ctrlFB, sysModel));
    eig(loopGain)

    % Stability Margins
    [Gm, Pm, Wcg, Wcp] = margin(loopGain);
    Gm_dB = 20*log10(Gm);
    disp(['GM: ', num2str(Gm_dB), '  dB @ ', num2str(Wcg), ' Hz']);
    disp(['PM: ', num2str(Pm), ' deg @ ', num2str(Wcp), ' Hz']);
    
    figure(Hz*10+3);
%     bode(loopGain); title([sysCL.Name, ' Loop Gain']);
%     grid on; hold on; 
    %semilogx([Wcg, Wcg], [0, Gm], 'r-'); hold on; 
    
    [mag,phase,wout] = bode(loopGain);
    subplot(2,1,1);
    semilogx(wout, 20*log10(squeeze(mag)), '-b'); hold on;
    semilogx([Wcg, Wcg], [0, Gm], 'r-'); hold on; 
    semilogx(wout, zeros(size(wout)), 'k--'); %xlim([1e-1, 1e5])
    grid on;
    
    subplot(2,1,2);
    semilogx(wout, squeeze(phase), '-b'); hold on;
    semilogx([Wcp, Wcp], [-180, -180+Pm], 'r-');
    semilogx(wout, -180*ones(size(wout)), 'k--'); %xlim([1e-1, 1e5])
    grid on;
     
    figure(Hz*10+4);
    nyquist(loopGain); title([sysCL.Name, ' Nyquist'])
    axis equal;    grid;

    % Sensitivity
    S = 1/(1 + loopGain);
    S_peak = getPeakGain(S);
    VGM = S_peak / (S_peak - 1);

    figure(Hz*10+5);
    bodemag(S); title([sysCL.Name, ' Sensitivity']);

    % Complementary Sensitivity
    T = feedback(loopGain, 1);

    figure(Hz*10+6);
    bodemag(T); title([sysCL.Name, ' Complementary Sensitivity']);
end