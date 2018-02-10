%%% Main Function for MAE277
%%% Min, Hosik, Preston

clc; clear all; close all;

%%%
% Motor Parameters
LinkLength = 0.0508;        % 2 inches due to mounting
LinkMass   = 0.045;         % 45 grams
Km         = 22.7E-3;       % 22.7 mNm/A
Vs         = 12;            % 12 V
R          = 8.0061;        % 8.0061 Ohm
L          = 0.24E-3;       % 0.24 mH
J_motor    = 0.00000103;    % 10.3 gcm^2
b          = 0;             % ~0 Nms
g          = 9.81;          % 9.81 m/s^2

J_link     = (1/3) * LinkMass * LinkLength ^ 2;
J_total    = J_motor + J_link;
kappa = Vs/Km;
tau = J_total*R/(Km^2);

%%%
% Continuous Time Model LTI State Space System
% Inverted Pendulum
A = [0, 1; (3*g)/(2*J_total), -3*(Km^2)/(LinkMass*(J_total^2)*R)];
B = [0; 3*Km*Vs/(LinkMass*(J_total^2)*R)];
C = [1, 0];
D = 0;

sys_c = ss(A,B,C,D);

% Controllability & Observability of the Continuous-Time
% Open Loop System
disp('Controllability of CT System');
disp(rank(ctrb(sys_c.A,sys_c.B)));
disp('Observability of CT System');
disp(rank(obsv(sys_c.A,sys_c.C)));

% Indirect Digital Control Design
[K_i,L_i,sys_id] = dcontrold_ind(sys_c);
% sys_id is the indirect design estimator feedback system

% Direct Digital Control Design
[K_d,L_d,sys_dd] = dcontrold_dir(sys_c);
% sys_dd is the direct design estimator feedback system 

%%% Analyze Indirect and Direct Control