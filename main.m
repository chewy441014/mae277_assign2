%%% Main Function for MAE277
%%% Min, Hosik, Preston

%% Initialize Env
clc; clear; close all;

%% Model Identification
% Motor Parameters
global LinkLength LinkMass Vs R_motor L_motor J_total b_motor g Km Kb
LinkLength = 75.39e-3;      % 75.30 mm
LinkMass   = 45.73E-3;       % 45.73 grams
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

pole_K = [ -40+10i, -40-10i];
pole_L = [-160+10i, -160-10i];

%% Controllability & Observability of the Continuous-Time
% Open Loop System
disp('Controllability of CT System');
disp(rank(ctrb(sys_c.A,sys_c.B)));
disp('Observability of CT System');
disp(rank(obsv(sys_c.A,sys_c.C)));

f = fullfile('data','id_data','id_data.csv');
Wr = error_bound(f);

%% Indirect Digital Control Design
for tdx = 1:length(T)
    Ts = T(tdx);
    [K_i,L_i,sys_id,N_id] = dcontrold_ind(sys_c, pole_K, pole_L, Ts);
    analysis(10+tdx, sys_c, sys_id, K_i, L_i, 1/Ts, Wr);
end

% sys_id is the indirect design estimator feedback system

%% Direct Digital Control Design
% for tdx = 1:length(T)
    tdx = 3;
    Ts = T(tdx);
    pole_K_DT = exp(pole_K * Ts);
    pole_L_DT = exp(pole_L * Ts);
    [K_d,L_d, N_d, sys_d, sys_CL] = dcontrold_dir(sys_c, pole_K_DT, pole_L_DT, Ts);
    analysis(20+tdx, sys_c, sys_CL, K_d, L_d, 1/Ts, Wr);

%% simulation
[ Ref_d, Time_d, y_lti_d, y_nl_d, u_lti_d, u_nl_d ] = simulation(sys_d, K_d, L_d, N_d);
[ Ref_i, Time_i, y_lti_i, y_nl_i, u_lti_i, u_nl_i ] = simulation(sys_c, K_i, L_i, N_id);

% Verify the LTI Direct Digital Control Design with Simulations Results
data_d = [Ref_d, Time_d, y_lti_d, y_nl_d, u_lti_d, u_nl_d];
data_i = [Ref_i, Time_i, y_lti_i, y_nl_i, u_lti_i, u_nl_i];
verify_design(data_d,30);
verify_design(data_i,40);

%% Problem # 7.a
% Track a 2 Hz sine wave without an internal model
% QKnm_sin = 10000*eye(2);
% RKnm_sin =0.01;
% QLnm_sin = 10;
% RLnm_sin = 0.01;
% [pole_K_DT_sine_nomodel, ~, ~] = dlqr(sys_d.A, sys_d.B, QKnm_sin, RKnm_sin);
% [~, pole_L_DT_sine_nomodel, ~, ~] = kalman(sys_d, QLnm_sin, RLnm_sin);
% [~, ~, ~, ~, sys_CLNoModel] = dcontrold_dir(sys_c, pole_K_DT_sine_nomodel, pole_L_DT_sine_nomodel, Ts);
% sineWaveTracker(sys_CLNoModel, 2, Ts, 61);
sineWaveTracker(sys_CL, N_d, 70, 2, Ts);

%% Problem # 7.b
idt = 70; % Plot indicator
[sys_CLModel, loopGainModel] = sysStateEstimatorFeedbackWithModel(sys_d, 2);

figure(idt*10 + 1);
bode(sys_CLModel); title('Closed-Loop Plant with Internal Model Frequency Response');

figure(idt*10 + 2);
step(sys_CLModel); title('Closed-Loop Plant with Internal Model Step Response');
stepInfoCLModel = stepinfo(sys_CLModel);

sineWaveTracker(sys_CLModel, 0, 71, 2, Ts);

analysisGivenLoopGain( loopGainModel, idt, 1/Ts);