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
x1_0 = pi;                              % Lambda
x2_0 = 0;                               % r
x3_0 = 0;                               % p
x4_0 = 0;                               % p_dot
x5_0 = 0;  
x6_0 = 0;  
x0 = [x1_0 x2_0 x3_0 x4_0 x5_0 x6_0]';          % Initial values

% Time horizon and initialization
N  = 40;                                % Time horizon for states
M  = N;                                 % Time horizon for inputs
z0  = zeros(N*mx+M*mu,1);                % Initialize z for the whole horizon
z0(1:mx) = x0;                           % Initial value for optimization

% Generate the matrix Q and the vector c (objecitve function weights in the QP problem) 
Q1 = zeros(mx,mx);
Q1(1,1) = 0.5;                             % Weight on state x1
Q1(2,2) = 0;                            % Weight on state x2
Q1(3,3) = 0;                             % Weight on state x3
Q1(4,4) = 0;                            % Weight on state x4
Q1(5,5) = 0;                            % Weight on state x5
Q1(6,6) = 0;                            % Weight on state x6
q1 = 0.5;                                 % Weight on input pitch
q2 = 0.5;                                 % Weight on input elevation
Q = 2*genq2(Q1,[q1, 0; 0, q2],N,M,mu);    % Generate Q

%% Generate system matrices for linear model
Aeq = gena2(A,B,N,mx,mu);  		 % Generate A, hint: gena2
beq = zeros(size(Aeq,1),1);
%beq = zeros(N*mx+M*mu,1);            % Generate b
beq(1:mx) = A*x0; % Initial value

%% Objectivefunction
objective = @(z) z'*Q*z;
options = optimset('MaxFunEvals',60000,'Algorithm', 'active-set');
z = fmincon(objective, z0,[],[],Aeq,beq,[],[],@nonlcon,options);


%% Extract control inputs and states

u1  = [z(N*mx+1:2:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution
u2  = [z(N*mx+2:2:N*mx+M*mu);z(N*mx+M*mu)]; % Control input from solution

x1 = [x0(1);z(1:mx:N*mx)];              % State x1 from solution
x2 = [x0(2);z(2:mx:N*mx)];              % State x2 from solution
x3 = [x0(3);z(3:mx:N*mx)];              % State x3 from solution
x4 = [x0(4);z(4:mx:N*mx)];              % State x4 from solution
x5 = [x0(5);z(5:mx:N*mx)];              % State x5 from solution
x6 = [x0(6);z(6:mx:N*mx)];              % State x6 from solution

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





