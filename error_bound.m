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
    h_frd = fft(yhat)./fft(u);
    w_final = 2; %Hz
    w = (0:w_final/(2*n-1):w_final);
    
    h_nom = squeeze(freqresp(ModelSys,w,'Hz'));
    
    mag_frd = 20*log10(abs(h_frd(1:n)));
    mag_nom = 20*log10(abs(h_nom(1:n)));
    mag_err = (mag_frd - mag_nom);
    w = w(1:n);

    %% uncertainty high pass filter

    HighPassOrder = 3;
    UpperBound = 16; 
    LowerBound = -0.5;   
    CrossFreq  = 0.004*2*pi;
    Wr = makeweight( db2mag(LowerBound), CrossFreq, db2mag(UpperBound) );
    Wr = Wr^HighPassOrder;
    h_Wr = squeeze(freqresp(Wr,w,'Hz'));
    mag_Wr = 20*log10(abs(h_Wr(1:n)));

    figure;
    semilogx(w,mag_frd,...
             w,mag_nom)
    grid on
    ylabel('Magnitude [dB]')
    legend('FRD','Model')

    figure;
    semilogx(w,mag_err,...
             w,mag_Wr)
    xlim([.1 500])
    ylim([-20 70])
    grid on
    legend('Magnitude Error','Wr(s)', 'location', 'SouthWest')
    ylabel('Magnitude [dB]')
    xlabel('Frequency [Hz]')
    
end