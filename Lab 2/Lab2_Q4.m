clc; clear; close all

% system properties
m = 1500; %[kg]
b = 50; %[N.s/m]

% A, B, C, D
A = [0 1; 0 -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;

% sim params
ts = 0.1;
t1 = 0:ts:100;
U = 100*ones(size(t1));

% create system
sys = ss(A,B,C,D);

% initial condition
x0 = [0,0];
initial(sys,x0);

% plot
[y,t,x] = lsim(sys,U,t1);

yyaxis left
plot(t,y)
ylabel('Output')
hold on
yyaxis right
plot(t,x(:,2))
ylabel('Velocity')