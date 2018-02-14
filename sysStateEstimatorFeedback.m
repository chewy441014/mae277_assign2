function [ sys_sefb ] = sysStateEstimatorFeedback( sys, K, L )
%SYSSTATEESTIMATORFEEDBACK Build the state estimator feedback matrix and return the augmented sys.
%   Given feedback gain K and observer gain L, return the augmented matrix
%   that represents the state estimator feedback system.

[A, B, C, D, Ts] = ssdata(sys);
As = [  A - B * K,      B * K           ;
        0               A - L * C       ];

Bs = [  B   ;
        0   ];

Cs = [  C,              0               ];

Ds = 0;

sys_sefb = ss(As, Bs, Cs, Ds, Ts);

end

