%%% Main Function for MAE277
%%% Min, Hosik, Preston

clc; clear; close all;

%%%
% Motor Parameters
LinkLength = 0.0508;        % 2 inches due to mounting
LinkMass   = 0.045;         % 45 grams
Km         = 22.7E-3;       % 22.7 mNm/A
Vs         = 12;            % 12 V
R_dd          = 8.0061;        % 8.0061 Ohm
L          = 0.24E-3;       % 0.24 mH
J_motor    = 0.00000103;    % 10.3 gcm^2
b          = 0;             % ~0 Nms
g          = 9.81;          % 9.81 m/s^2

J_link     = (1/3) * LinkMass * LinkLength ^ 2;
J_total    = J_motor + J_link;
kappa = Vs/Km;
tau = J_total*R_dd/(Km^2);
T = [1/10, 1/100, 1/1000];
n = length(T);

%%%
% Continuous Time Model LTI State Space System
% Inverted Pendulum
A = [0, 1; (3*g)/(2*J_total), -3*(Km^2)/(LinkMass*(J_total^2)*R_dd)];
B = [0; 3*Km*Vs/(LinkMass*(J_total^2)*R_dd)];
C = [1, 0];
D = 0;

sys_c = ss(A,B,C,D);

% Controllability & Observability of the Continuous-Time
% Open Loop System
disp('Controllability of CT System');
disp(rank(ctrb(sys_c.A,sys_c.B)));
disp('Observability of CT System');
disp(rank(obsv(sys_c.A,sys_c.C)));

%%% Indirect Digital Control Design
% [K_i,L_i,sys_id] = dcontrold_ind(sys_c);
% sys_id is the indirect design estimator feedback system

%%% Direct Digital Control Design
[K_d,L_d,sys_dd,Lz_d] = dcontrold_dir(sys_c);
% sys_dd is the direct design estimator feedback system 
sys_dd{1}.Name = [num2str(1/T(1)),' Hz'];
sys_dd{2}.Name = [num2str(1/T(2)),' Hz'];
sys_dd{3}.Name = [num2str(1/T(3)),' Hz'];

%%% Analyze Indirect and Direct Control
% Step Response
opt = stepDataOptions('StepAmplitude',0.2);
step(sys_dd{1},sys_dd{2},sys_dd{3},opt); title('Step Response Comparison');
legend(sys_dd{1}.Name,sys_dd{2}.Name,sys_dd{3}.Name,'Location','southeast')

% Loop Gain
S_dd = cell(1,n);
PeakS_dd = cell(1,n);
VGM_dd = cell(1,n);
R_dd = cell(1,n);
for i = 1:n
    figure; bode(Lz_d{i}); title([sys_dd{i}.Name,' Closed Loop Bode']);
    figure; nyquist(Lz_d{i}); axis equal; title([sys_dd{i}.Name,' Nyquist']);
    %Sensitivity
    S_dd{i} = 1/(1 + Lz_d{i}); PeakS_dd{i} = getPeakGain(S_dd{i});
    %Robustness
    R_dd{i} = Lz_d{i}/(1 + Lz_d{i});
    %Vector Gain Margin
    VGM_dd{i} = PeakS_dd{i}/(PeakS_dd{i} - 1); disp(['VGM is ',num2str(VGM_dd{i}), ' for ',sys_dd{i}.Name,' case']);
    figure; bodemag(S_dd{i});  title([sys_dd{i}.Name,' Sensitivity']);
    figure; bodemag(R_dd{i});  title([sys_dd{i}.Name,' Robustness']);
end