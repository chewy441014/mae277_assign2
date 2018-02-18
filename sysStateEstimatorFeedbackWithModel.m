function [ sysCL, Lz ] = sysStateEstimatorFeedbackWithModel( sys, w )
%SYSSTATEESTIMATORFEEDBACKWITHMODEL Build the state estimator feedback
%matrix with an internal model included.
%   Detailed explanation goes here

% Extract the discrete-time state-space model and sampling rate of the plant
[Ap, Bp, Cp, Dp, Ts] = ssdata(sys);

nStates = size(Ap, 1);
nInputs = size(Bp, 2);
nOutputs = size(Cp, 1);


%% =============================== %%
%% Build the internal model for tracking a w Hz sinusoidal wave
Am = [  0       1                   ;
        -1      2*cos(2*pi*w*Ts)    ];

Bm = [  0   ;
        1   ];
    
Cm = [  1   0   ];

Dm = 0;

% Am = 1;
% Bm = 1;
% Cm = 1;
% Dm = 0;

sys_M = ss(Am, Bm, Cm, Dm, Ts);

nStatesModel = size(Am, 1);
nInputsModel = size(Bm, 2);
nOutputsModel = size(Cm, 1);


%% =============================== %%
%% Build the augmented system matrices
Aaug = [    Ap,             zeros(nStates, nStatesModel)   ;
            Bm*Cp,          Am                              ];

Baug = [    Bp                              ;
            zeros(nStatesModel, nInputs)   ];

Baugr = [   zeros(nStates, nInputsModel)    ;
            -Bm                             ];

Caug = [    Cp,             zeros(nOutputsModel, nStatesModel)   ];

%% =============================== %%
%% State Feedback Controller
Qfb = [ 10,      0,          0,          0   ;
        0,          0.001,      0,          0   ;
        0,          0,          10,      0   ;
        0,          0,          0,          0.001  ];
% Qfb = [10000, 0, 0; 0, 1000, 0; 0, 0, 10];
Rfb = 100;
[Kaug, ~, ~] = dlqr(Aaug, Baug, Qfb, Rfb);
K = Kaug(1:nStates);
Km = Kaug(nStates + 1:end);

%% =============================== %%
%% State Observer
Qob = 10;
Rob = 1;

[~, L, ~, ~] = kalman(sys, Qob, Rob);

%% =============================== %%
%% Closed-Loop System %%
Acl = [ Aaug - Baug*Kaug,                       Baug*K          ;
        zeros(nStates, nStates+nStatesModel)    Ap - L * Cp     ];

Bcl = [ Baug + Baugr                ;
        zeros(nStates, nInputs)     ];
    
Ccl = [ Caug,       zeros(nOutputs, nStates)    ];

Dcl = 0;

sysCL = minreal(ss(Acl, Bcl, Ccl, Dcl, Ts));


%% =============================== %%
%% Loop Gain %%
Lz = - ss(Ap - L*Cp - Bp*K, Bp, K, 0, Ts) * ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     + ss(Ap - L*Cp - Bp*K, L, K, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     + ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts);

Lz2 = ss(Ap - L*Cp - Bp*K, Bp, K, 0, Ts) * ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     + ss(Ap - L*Cp - Bp*K, L, K, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     - ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts);

Laug = [    L       ;
            Bm      ];
 
Lz3 = ss(Aaug - Laug*Caug - Baug*Kaug, Laug, Kaug, 0, Ts)*ss(Aaug, Baug, Caug, 0, Ts);
end

