% TTK4135 - Helicopter lab
 %problem 3.

%% Initialization and model definition
problem2
%ex2_4
%% Drive force penalty

R = [0.1];

Q_feedback = diag([1, 0,0,0]);

%% Calculate K
[K, P, e] = dlqr(A1, B1, Q_feedback, R, []);

%% Export to Simulink
pitchRef = [t', u];
travelRef = [t', x1, x2, x3, x4];


plot(t',x1,t',u);

