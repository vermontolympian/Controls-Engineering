clc; clear; close all

% system params
m = 0.1; % [kg] mass of cart
M = 1; % [kg] mas of pendulum
L = 1; % [m] Length of pendulum
g = 9.8; % [m/s^2] Gravity
u = 0; %control input Force
I = 0.006; %kg.m^2 Moment of interia 
b= 0.1; %coefficent of Friction probably needs to be changed

p = I*(M+m)+M*m*1^2; % denomiator for the A and B matrices


% A, B, C, D
A = [0 1 0 0; 
    0 -(I+m*L^2)*b/p (m^2*g*L^2)/p 0; 
    0 0 0 1; 
    0 -(m*L*b)/p       m*g*L*(M+m)/p  0];
B = [0;
    (I+m*L^2)/p; 
    0; 
    m*L/p];
C = [1 0 0 0;
    0 0 1 0];
D = [0 ; 0]; 

syms k1 k2 k3 k4 lambda

K = [k1, k2, k3, k4];

lambdas = [-1, -2, -1+i, -1-i];

MM = A-B*K;

% eigs = eig(MM)

eq = det(A-B*K-lambda*eye(4));

equ1 = subs(eq, lambda, lambdas(1)) == 0;

equ2 = subs(eq, lambda, lambdas(2)) == 0;

equ3 = subs(eq, lambda, lambdas(3)) == 0;

equ4 = subs(eq, lambda, lambdas(4)) == 0;

equ = [equ1, equ2, equ3, equ4];

S = solve(equ, K);

k_1 = S.k1;
k_2 = S.k2;
k_3 = S.k3;
k_4 = S.k4;

k = [k_1; k_2; k_3; k_4;]