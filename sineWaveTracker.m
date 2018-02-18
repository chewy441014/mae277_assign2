function sineWaveTracker( sys, w, Ts, idt, tend)
%SINEWAVETRACKER Track a given sinusoidal reference trajectory.
%   Given a system, track a sinusoid with a frequency of w at Ts for tend
%   time.

if nargin < 5 % If ending time is not specified
    tend = 10;
end

% Construct the sinusoidal input
amp = 0.1
t = [0:Ts:tend];
r = amp*sin(2*pi*w*t);

% Run the simulation
[y, t, ~] = lsim(sys, r, t);

% Plot the reference input and the output
figure(idt);
plot(t, y, 'o-', t, r, '.-'); 
title(['Reference Tracking of a ', num2str(w), ' Hz Sine Wave at ', num2str(1/Ts), ' Hz']);
xlabel('Time [s]'); ylabel('Position [rad]'); 
ylim([-1.5*amp, 1.5*amp]);
legend('Response','Reference');

end

