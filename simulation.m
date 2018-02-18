function [ Ref, Time, y_lti, y_nl, u_lti, u_nl ] = simulation(sys, K, L, N)
global LinkLength LinkMass Vs R_motor L_motor J_total b_motor g Km Kb

QzrLvl    = 2*pi/400;

K, L, N

sysP = sys;
A  = sys.A;
B  = sys.B;
C  = sys.C;
D  = sys.D;
Ts = sys.Ts;
if Ts == 0
    Ts = 1/1000; % For indirect design we only consider the ...
    %controller designed for this loop rate
    modelname = 'nonlinear_inverted_pendulum_simulation_indirect';
    options = simset('SrcWorkspace','current');%'debug', 'on', 
else
    modelname = 'nonlinear_inverted_pendulum_simulation_direct';
    options = simset('SrcWorkspace','current');
    
end

nInputs  = size(B,2);
nOutputs = size(C,1);
Ctrl_FB = minreal( -tf( ss(A - L*C - B*K, L, K, zeros(nInputs,nOutputs) ) ) );

% run simulation
open(modelname);
sim(modelname,[],options);

% collect data
Ref = ref.Data;
Time = ref.Time;
y_lti = y_LTI.Data;
y_nl = y_NL.Data;
u_lti = u_LTI.Data;
u_nl = u_NL.Data;

end

