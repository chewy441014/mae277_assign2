function Lz= LoopGain(sys, L, K)
% Get loop gain given the system matrices and the gains
A = sys.A;
B = sys.B;
C = sys.C;
Ts = sys.Ts;
if Ts > 0
    %Discrete time system loop gain
    Lz = ss(A-L*C-B*K, L, K, 0, Ts)*ss(A, B, C, 0, Ts);
else
    %Continuous time system loop gain
    Lz = ss(A-L*C-B*K, L, K, 0)*ss(A, B, C, 0);
end
