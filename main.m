%%% Main Function for MAE277
%%% Min, Hosik, Preston

%% Initialize Env
clc; clear; close all;

%% Model Identification
% Motor Parameters
LinkLength = 75.39e-3;      % 75.30 mm
LinkMass   = 45.73-3;       % 45.73 grams
% Km       = 22.7E-3;       % 22.7 mNm/A
Vs         = 12;            % 12 V
% R_dd     = 8.0061;        % 8.0061 Ohm
% L        = 0.24E-3;       % 0.24 mH
J_motor    = 0.00000103;    % 10.3 gcm^2
% b        = 0;             % ~0 Nms
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
R = tau_id*Km^2 / J_total

T = [1/10, 1/100, 1/1000];
%% Dynamic Model, Matrix Representation
% Continuous Time Model LTI State Space System
% Inverted Pendulum
% A = [0, 1; (3*g)/(2*J_total), -3*(Km^2)/(LinkMass*(J_total^2)*R_dd)];
% B = [0; 3*Km*Vs/(LinkMass*(J_total^2)*R_dd)];
% C = [1, 0];
% D = 0;

A = [                   0,                                      1; 
     (3*g)/(2*LinkLength), (-3*Km*Kb)/(LinkMass*(LinkLength^2)*R)];
B = [                   0; ( 3*Km*Vs)/(LinkMass*(LinkLength^2)*R)];
C = [                   1,                                      0];
D =                                                             0;

sys_c = ss(A,B,C,D);

%% Controllability & Observability of the Continuous-Time
% Open Loop System
disp('Controllability of CT System');
disp(rank(ctrb(sys_c.A,sys_c.B)));
disp('Observability of CT System');
disp(rank(obsv(sys_c.A,sys_c.C)));

%% Indirect Digital Control Design
% [K_i,L_i,sys_id] = dcontrold_ind(sys_c);
% sys_id is the indirect design estimator feedback system

%% Direct Digital Control Design
Kp = [0.8+0.15i, 0.8-0.15i];
Lp = Kp/10;

K_d = cell(1,length(T));
L_d = cell(1,length(T));
sys_dd = cell(1,length(T)); 
Lz_d = cell(1,length(T));   % Loop Gain

S_dd = cell(1,length(T));   % Sensitivity
PeakS_dd = cell(1,length(T)); % Peak value of Sensitivity
VGM_dd = cell(1,length(T)); % Vector Gain Margin
R_dd = cell(1,length(T));   % Robustness

%%% Analyze Indirect and Direct Control
for tdx = 1:length(T)
    Ts = T(tdx);
    [K_d{tdx},L_d{tdx},sys_dd{tdx},Lz_d{tdx}] = dcontrold_dir(sys_c, Kp, Lp, Ts);
    S_dd{tdx} = 1/(1 + Lz_d{tdx}); PeakS_dd{tdx} = getPeakGain(S_dd{tdx});
    R_dd{tdx} = Lz_d{tdx}/(1 + Lz_d{tdx});
    VGM_dd{tdx} = PeakS_dd{tdx}/(PeakS_dd{tdx} - 1); 
    
    % sys_dd is the direct design estimator feedback system 
    sys_dd{tdx}.Name = [num2str(1/T(tdx)),' Hz'];
    disp(['VGM is ',num2str(VGM_dd{tdx}), ' for ',sys_dd{tdx}.Name,' case']);
    
    % TODO: figure numbering
    figure; bode(Lz_d{tdx}); title([sys_dd{tdx}.Name,' Closed Loop Bode']);
    figure; nyquist(Lz_d{tdx}); axis equal; title([sys_dd{tdx}.Name,' Nyquist']);
    figure; bodemag(S_dd{tdx});  title([sys_dd{tdx}.Name,' Sensitivity']);
    figure; bodemag(R_dd{tdx});  title([sys_dd{tdx}.Name,' Robustness']);
end

% Step Response % TODO
opt = stepDataOptions('StepAmplitude',0.2);
step(sys_dd{1},sys_dd{2},sys_dd{3},opt); title('Step Response Comparison');
legend(sys_dd{1}.Name,sys_dd{2}.Name,sys_dd{3}.Name,'Location','southeast')