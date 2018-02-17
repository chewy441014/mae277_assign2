function verify_design(data,idt)
% :param data: contains the output of simulation
% :param idt: plot index
%  Ref_d, Time_d, y_lti_d, y_nl_d, u_lti_d, u_nl_d 

ref = data(:,1);
t = data(:,2);
y = data(:,3);
ynl = data(:,4);
u = data(:,5);
unl = data(:,6);

n = min([length(ref),length(t),length(y),length(ynl),length(u),length(unl)]);

t = t(1:n);
y = y(1:n);
ynl = ynl(1:n);
u = data(1:n);
unl = unl(1:n);

figure(idt+1);
plot(t,y,t,ynl); title('Control Simulation Output Comparison'); 
xlabel('Time (s)'); ylabel('Theta (rad)');

figure(idt+2);
plot(t,u,t,unl); title('Control Simulation Control Effort Comparison'); 
xlabel('Time (s)'); ylabel('Control Effort');

end