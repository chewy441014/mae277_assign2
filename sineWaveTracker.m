function sineWaveTracker( sys, w, Ts, tend )
%SINEWAVETRACKER Track a given sinusoidal reference trajectory.
%   Given a system, track a sinusoid with a frequency of w at Ts for tend
%   time.

if nargin < 4 % If ending time is not specified
    tend = 10;
end

% Construct the sinusoidal input
t = [0:Ts:tend];
r = sin(2*pi*w*t);

% Run the simulation
[y, t, ~] = lsim(sys, r, t);

% Plot the reference input and the output
plot(t, y, 'g', t, r, 'r-'); title(['Reference Tracking of a ', num2str(w), ' Hz Sine Wave at ', num2str(1/Ts), ' Hz']);
xlabel('Time [s]'); ylabel('Position [rad]');

end

