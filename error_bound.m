function Wr = error_bound(filename)
	%% ============================== %%
    % :param filename: file name of chirp response .csv file
    % :param ModelSys: model of the LTI system (open loop)
    
    data = importdata(filename);
    t = data(:,1);
    y = data(:,3);
    u = data(:,4);
    
    n = round(length(t)/2);
    
    % filter the data
    d = designfilt('lowpassiir','FilterOrder',6,'HalfPowerFrequency',0.055);
    yhat=filtfilt(d,y);
    
    % System Model
    ModelSys = tf([0,451.4583],[0.2009, 1]);

    % frequency response 
    h_frd = fft(yhat)./fft(u); h_frd = h_frd(2:end); %first value crop
    w_final = 200; %Hz
    w = (0:w_final/(2*n-1):w_final); w = w(1:(end-2));
    
    h_nom = squeeze(freqresp(ModelSys,w,'Hz'));
    
    mag_frd = 20*log10(abs(h_frd));
    mag_nom = 20*log10(abs(h_nom));
    mag_err = abs(mag_frd - mag_nom);
    w = w(1:n);
    
    %mag_frd_adj = mag_frd + mean(mag_err(1:10));
    %mag_err_adj = abs(mag_frd_adj - mag_nom);
    %% uncertainty high pass filter

    HighPassOrder = 3;
    UpperBound = 11; 
    LowerBound = -0.1;   
    CrossFreq  = 0.0075*2*pi;
    Wr = makeweight( db2mag(LowerBound), CrossFreq, db2mag(UpperBound) );
    Wr = Wr^HighPassOrder;
    h_Wr = squeeze(freqresp(Wr,w,'Hz'));
    mag_Wr = 20*log10(abs(h_Wr));

    figure;
    semilogx(w,mag_frd(1:n),...
             w,mag_nom(1:n))%,w,mag_frd_adj(1:n))
    grid on
    ylabel('Magnitude [dB]')
    legend('FRD','Model')

    figure;
    semilogx(w,mag_err(1:n),...
             w,mag_Wr)%,w,mag_err_adj(1:n))
    xlim([.1 500])
    ylim([-20 70])
    grid on
    legend('Magnitude Error','Wr(s)', 'location', 'SouthWest')
    ylabel('Magnitude [dB]')
    xlabel('Frequency [Hz]')
    
end