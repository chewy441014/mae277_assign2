function [ sys_CL, Lz ] = sysStateEstimatorFeedbackWithModel( sys, w )
%SYSSTATEESTIMATORFEEDBACKWITHMODEL Build the state estimator feedback
%matrix with an internal model included.
%   Detailed explanation goes here

% Extract the state-space model and sampling rate of the plant
[Ap, Bp, Cp, Dp, Ts] = ssdata(sys)

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

Caug = [    C,              zeros(nOutputsModel, nStatesModel)   ];

%% =============================== %%
%% State Feedback Controller
Qfb = [ 100000*eye(2),  zeros(2,2)      ;
        zeros(2,2),     10000*eye(2)    ];
Rfb = 100000;
[Kaug, ~, ~] = dlqr(Aaug, Baug, Qfb, Rfb);
K = Kaug(1:nStates);
Km = Kaug(nStates + 1:end);

%% =============================== %%
%% State Observer
Qob = 10;
Rob = 0.01;

[~, L, ~, ~] = kalman(sys, Qob, Rob);

%% =============================== %%
%% Closed-Loop System %%
Acl = [ Aaug - Baug*Kaug,                       Baug*K          ;
        zeros(nStates, nStates+nStatesModel)    Ap - L * Ac     ];

Bcl = [ Baug + Baugr    ;
        zeros(nStates)  ];
    
Ccl = [ Caug,       zeros(nOutputs, nStates)    ];

Dcl = 0;

sysCL = ss(Acl, Bcl, Ccl, Dcl, Ts);

%% =============================== %%
%% Loop Gain %%
Lz = - ss(Ap - L*Ap - Bp*K, Bp, K, 0, Ts) * ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     + ss(Ap - L*Cp - Bp*K, L, K, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts) ...
     + ss(Am, Bm, Km, 0, Ts) * ss(Ap, Bp, Cp, 0, Ts);
 

end

