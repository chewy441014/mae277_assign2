function [ sys_sefb ] = sysStateEstimatorFeedback( sys, K, L )
%SYSSTATEESTIMATORFEEDBACK Build the state estimator feedback matrix and return the augmented sys.
%   Given feedback gain K and observer gain L, return the augmented matrix
%   that represents the state estimator feedback system.

[A, B, C, D, Ts] = ssdata(sys);

nStates = size(A, 1);
nInputs = size(B, 2);
nOutputs = size(C, 1);

As = [  A - B * K,              B * K           ;
        zeros(nStates)          A - L * C       ];

Bs = [  B   ;
        zeros(nStates, nInputs)   ];

Cs = [  C,                      zeros(nOutputs, nStates)  ];

Ds = 0;

sys_sefb = ss(As, Bs, Cs, Ds, Ts);

end

