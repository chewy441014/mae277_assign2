%%% Main Function for MAE277
%%% Min, Hosik, Preston

%% Initialize Env
clc; clear; close all;

%% Model Identification
% Motor Parameters
global LinkLength LinkMass Vs R_motor L_motor J_total b_motor g Km Kb
LinkLength = 75.39e-3;      % 75.30 mm
LinkMass   = 45.73-3;       % 45.73 grams
% Km       = 22.7E-3;       % 22.7 mNm/A
Vs         = 12;            % 12 V
% R_motor    = 8.0061;        % 8.0061 Ohm
L_motor    = 0.24E-3;       % 0.24 mH
J_motor    = 0.00000103;    % 10.3 gcm^2
b_motor    = 0;             % ~0 Nms
g          = 9.81;          % 9.81 m/s^2

J_link     = (1/3) * LinkMass * LinkLength ^ 2;
J_total    = J_motor + J_link;
% kappa = Vs/Km;
% tau = J_total*R_dd/(Km^2);

% From System Id
% [kappa_id, tau_id] = system_id()
kappa_id = 451.4583;
tau_id = 0.2009;

% Calc unknown parameters
Km = Vs/kappa_id;
Kb = Km;
R_motor = tau_id*Km^2 / J_total;

T = [1/10, 1/100, 1/1000];
%% Dynamic Model, Matrix Representation
% Continuous Time Model LTI State Space System Inverted Pendulum
A = [                   0,                                            1; 
     (3*g)/(2*LinkLength), (-3*Km*Kb)/(LinkMass*(LinkLength^2)*R_motor)];
B = [                   0; ( 3*Km*Vs)/(LinkMass*(LinkLength^2)*R_motor)];
C = [                   1,                                            0];
D =                                                                   0;

sys_c = ss(A,B,C,D); % poles @  11.7020, -16.6797

% pole_K = [-40+10j, -40-10j];
% pole_L = [-160+10j, -160-10j];%pole_K*10;
pole_K = [-200+10j, -200-10j];
pole_L = [-2400+10j, -2400-10j];%pole_K*10;

%% Controllability & Observability of the Continuous-Time
% Open Loop System
disp('Controllability of CT System');
disp(rank(ctrb(sys_c.A,sys_c.B)));
disp('Observability of CT System');
disp(rank(obsv(sys_c.A,sys_c.C)));

%% Indirect Digital Control Design
[K_i,L_i,sys_id] = dcontrold_ind(sys_c, pole_K, pole_L);
analysis(10, sys_c, sys_id, K_i, L_i);

% sys_id is the indirect design estimator feedback system

%% Direct Digital Control Design
for tdx = 1:length(T)
    Ts = T(tdx);
    Ts = 1/1000;
    pole_K_DT = exp(pole_K * Ts);
    pole_L_DT = exp(pole_L * Ts);
    [K_d,L_d, N_d, sys_d, sys_CL] = dcontrold_dir(sys_c, pole_K_DT, pole_L_DT, Ts);
    
    analysis(20, sys_c, sys_CL, K_d, L_d, 1/Ts);
end
    

%% simulation
[ Ref, Time, y_lti, y_nl, u_lti, u_nl ] = simulation(sys_d, K_d, L_d, N_d);

% Step Response % TODO
% opt = stepDataOptions('StepAmplitude',0.2);
% step(sys_dd{1},sys_dd{2},sys_dd{3},opt); title('Step Response Comparison');
% legend(sys_dd{1}.Name,sys_dd{2}.Name,sys_dd{3}.Name,'Location','southeast')


%% Problem # 7.a

% State estimator feedback matrix
sys_sefb = sysStateEstimatorFeedback(sys_d, K_d, L_d); % Unnecessary since dcontrol_dir() now also returns the CL sys

% Track a 2 Hz sine wave without an internal model
sineWaveTracker(sys_CL, 2, Ts);

%% Problem # 7.b
idt = 70; % Plot indicator
[sys_CLModel, loopGainModel] = sysStateEstimatorFeedbackWithModel(sys_d, 2);

figure(idt*10 + 1);
bode(sys_CLModel); title('Closed-Loop Plant with Internal Model Frequency Response');

figure(idt*10 + 2);
step(sys_CLModel); title('Closed-Loop Plant with Internal Model Step Response');
stepInfoCLModel = stepinfo(sys_CLModel);

figure(idt*10 + 7);
sineWaveTracker(sys_CLModel, 2, Ts);

analysisGivenLoopGain( loopGainModel, 70, 1/Ts);