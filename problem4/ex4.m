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
       0, K_3*Kep];
   
%% Discrete time system
A = eye(6) + delta_t * A_c;
B = delta_t * B_c;