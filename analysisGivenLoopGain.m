function analysisGivenLoopGain( loopGain, idt , Hz, Wr)
%ANALYSISGIVENLOOPGAIN Given loop again, conduct the analysis.
%   With loop gain as the input, analyze the stability, transient response,
%   steady state response, and robust stability.
%   :param loopGain: Loop gain
%   :param idt: Indicator value for smart plotting
%   :param Hz: frequency label for plot titles

    % Stability Margins
    [Gm, Pm, Wcg, Wcp] = margin(loopGain);
    Gm_dB = 20*log10(Gm);
    disp(['GM: ', num2str(Gm_dB), '  dB @ ', num2str(Wcg), ' Hz']);
    disp(['PM: ', num2str(Pm), ' deg @ ', num2str(Wcp), ' Hz']);
    
    figure(idt*10+3);
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
     
    figure(idt*10+4);
    nyquist(loopGain); title(['Nyquist'])
    axis equal;    grid;

    % Sensitivity
    S = 1/(1 + loopGain); % feedback(1,loopGain);
    S_peak = getPeakGain(S);
    VGM = S_peak / (S_peak - 1)

    figure(idt*10+5);
    bodemag(S); title(['Sensitivity']);

    % Complementary Sensitivity
    T = 1-S; % feedback(loopGain, 1);

    figure(idt*10+6);
    bodemag(T); title(['Complementary Sensitivity']);
    
    % Error Bound
    if nargin > 3
        if Hz == 0
            figure(idt*10+7); bodemag(T, 1/Wr, 'r--'); legend('T(z)','1/Wr(z)'); title('Robust Stability');
        else
            Wr_d = c2d(Wr, 1/Hz, 'Tustin');
            figure(idt*10+7); bodemag(T, 1/Wr_d, 'r--'); legend('T(z)','1/Wr(z)'); title([num2str(Hz),' Hz Robust Stability']);
        end
    end
end

