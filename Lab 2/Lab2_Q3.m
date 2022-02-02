clc; clear; close all

% system params
m = 5;  %[kg]
k = 1;  %[N/m]
b = 0.5;  %[N.s/m]

% A, B, C, D
A = [0, 1; -k/m, -b/m];
B = [0; 1/m];
C = [1 0];
D = 0;


% sim params
ts = 0.1;
t1 = 0:ts:100;
F = 2*ones(size(t1));
%F = zeros(size(t1));
%F(1) = 2;


% create system
sys = ss(A,B,C,D);

% initial condition
x0 = [0,0];
initial(sys,x0);

% plot
[y,t,x] = lsim(sys,F,t1);

yyaxis left
plot(t,y)
ylabel('Output')
hold on
yyaxis right
plot(t,x(:,2))
ylabel('Velocity')