
close all;
clc;

A_c=[0,  1,      0,       0;
     0,  0,    -K_2,      0;
     0,  0,      0,       1;
     0,  0, -K_1*K_pp, -K_1*K_pd;
    ];

B_c = [ 0;
        0;
        0;
        K_1*K_pp;
    ];

%Diskretice
h = 0.25; % s    timestep
A = eye(4)+h*A_c;
B = h*B_c;

%Optimalisation problem
N = 100;  %Horizon
x_l=[-inf,-inf,-inf,-inf]';
x_u=[inf,inf,inf,inf]';
u_l=-pi/6;
u_u=pi/6;
lambda_0 = pi;
lambda_f = 0;
x_0=[lambda_0, 0, 0, 0]';
x_f=[lambda_f, 0, 0, 0]';
q=1;
Q=[2,0,0,0;
   0,0,0,0;
   0,0,0,0;
   0,0,0,0;
   ];
R=2*q;

G=genq2(Q,R,N,N,1);
A_eq=gena2(A,B,N,4,1);
B_eq=zeros(400,1);
B_eq(1:4)=A*x_0;
[l_b,u_b] = genbegr2(N,N,x_l,x_u,u_l,u_u);

X_opt=quadprog(G,zeros(1,500),A,B,A_eq,B_eq,l_b,u_b,x_0);



