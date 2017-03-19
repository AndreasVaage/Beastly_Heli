R = diag([1, 1]);
Q_feedback = diag([100, 0,0,0,100,0]);
t = 0:delta_t:delta_t*(length(u1)-1);
%% Calculate K
[K, P, e] = dlqr(A1, B1, Q_feedback, R, []);

%% Export to Simulink
pitchRef = [t', u1];
elevationRef = [t', u2];
figure(1)
plot([1:size(x1,1)]*delta_t,x1*180/pi,[1:size(x1,1)]*delta_t,x5*180/pi);
figure(2)
plot([1:size(u1,1)]*delta_t,u1*180/pi,[1:size(u2,1)]*delta_t,u2*180/pi);
