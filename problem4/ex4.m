init;
delta_t = 0.25;

%% Continuous time system
A_c = [0, 1, 0, 0, 0, 0;
       0, 0, -K_2, 0, 0, 0;
       0, 0, 0, 1, 0, 0;
       0, 0, -K_1*K_pp, -K_1*K_pd, 0, 0;
       0, 0, 0, 0, 0, 1;
       0, 0, 0, 0, -K_3*K_ep, -K_3*K_ed];
   
B_c = [0, 0;
       0, 0;
       0, 0;
       K_1*K_pp, 0;
       0, 0;
       0, K_3*K_ep];
   
%% Discrete time system
A = eye(6) + delta_t * A_c;
B = delta_t * B_c;

% Number of states and inputs
mx = size(A,2); % Number of states (number of columns in A)
mu = size(B,2); % Number of inputs(number of columns in B)

% Initial values
x1_0 = pi;      % Lambda
x2_0 = 0;       % r
x3_0 = 0;       % p
x4_0 = 0;       % p_dot
x5_0 = 0;  
x6_0 = 0;  
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]'; % Initial values

%% Time horizon and initialization
N  = 40;          % Time horizon for states
M  = N;           % Time horizon for inputs
z0  = zeros(N*mx+M*mu,1); % Initialize z for the whole horizon
z0(1:mx) = x0;    % Initial value for optimization

%% Bounds
upl = -20 * pi / 180; % Pitch lower
upu = 20 * pi / 180; % Pitch upper

uel = -60 * pi / 180; % Elevation lower
ueu = 60 * pi / 180; % Elevation upper

xl = -inf * ones(mx, 1);
xu = inf * ones(mx, 1);
[vlb, vub] = genbegr2(N, N, xl, xu, [upl, uel]', [upu, ueu]');
vub(length(vub)-mu*N-mx:length(vub)-mu*N) = mx*[0];
vlb(length(vlb)-mu*N-mx:length(vlb)-mu*N) = mx*[0];

%% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
Q1 = zeros(mx,mx);
Q1(1,1) = 0.5;   % Weight on travel
Q1(2,2) = 1;  % Weight on travel dot
Q1(3,3) = 0;  % Weight on pitch
Q1(4,4) = 1;  % Weight on pitch dot
Q1(5,5) = 1;  % Weight on elevation
Q1(6,6) = 1;  % Weight on elevation dot

q1 = 0.5;     % Weight on input pitch
q2 = 0.5;     % Weight on input elevation

Q = 2*genq2(Q1,[q1, 0; 0, q2],N,M,mu); % Generate Q

%% Generate system matrices for linear model
Aeq = gena2(A,B,N,mx,mu);  % Generate A
beq = zeros(size(Aeq,1),1);
%beq = zeros(N*mx+M*mu,1); % Generate b
beq(1:mx) = A*x0; % Initial value


%% Objectivefunction
objective = @(z) z'*Q*z;
options = optimset('MaxFunEvals',60000,'Algorithm', 'active-set');
z = fmincon(objective, z0,[],[],Aeq,beq,vlb,vub,@nonlcon,options);





%% Extract control inputs and states

u1  = [z(N*mx+1:2:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution
u2  = [z(N*mx+2:2:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution

x1 = [x0(1);z(1:mx:N*mx)];  % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];  % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];  % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];  % State x4 from solution
x5 = [x0(5);z(5:mx:N*mx)];  % State x5 from solution
x6 = [x0(6);z(6:mx:N*mx)];  % State x6 from solution

num_variables = 5/delta_t;
zero_padding = zeros(num_variables,1);
unit_padding  = ones(num_variables,1);

u1   = [zero_padding; u1; zero_padding];
u2   = [zero_padding; u2; zero_padding];
x1  = [pi*unit_padding; x1; zero_padding];
x2  = [zero_padding; x2; zero_padding];
x3  = [zero_padding; x3; zero_padding];
x4  = [zero_padding; x4; zero_padding];
x5  = [zero_padding; x5; zero_padding];
x6  = [zero_padding; x6; zero_padding];




%% Because I don't want to edit two files
R = diag([1, 1]);
Q_feedback = diag([100, 0,0,0,100,0]);
t = 0:delta_t:delta_t*(length(u1)-1);
%% Calculate K
[K, P, e] = dlqr(A, B, Q_feedback, R, []);

%% Export to Simulink
pitchRef = [t', u1];
elevationRef = [t', u2];
figure(1)
plot([1:size(x1,1)]*delta_t,x1*180/pi,[1:size(x1,1)]*delta_t,x5*180/pi);
figure(2)
plot([1:size(u1,1)]*delta_t,u1*180/pi,[1:size(u2,1)]*delta_t,u2*180/pi);

%% Contraint
l = linspace(70,170)*pi/180;
a = 0.2;
b = 20;
l_b = 2*pi/3;

e = a*exp(-b*(x1-l_b).^2);

l_deg = l*180/pi;
e_deg = e*180/pi;

figure(3)
hold on;
xlabel('Travel [deg]');
ylabel('Elevation [deg]');
plot(x1*180/pi,e_deg,'red');
plot(x1*180/pi,x5*180/pi);
legend('Mountain','Trajectory');

%%%%
[c, c_eq] = nonlcon(z);
%length(x1*180/pi)
%length([zero_padding;c;zero_padding;0])

handles(1) = xlabel('$\lambda$/degrees');
handles(2) = ylabel('$e$/degrees');
set(handles, 'Interpreter', 'Latex');



