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

syms K k1 k2 k3 k4 lambda

Ks = [0, -0.909, -3.1844, 3.1759];

% det(A-B*K-lambda*I)=0;

equ1 = 0 == det(A-B*K-k1*I);
subs(equ1, k1, Ks(1));
equ2 = 0 == det(A-B*K-k2*I);
subs(equ2, k2, Ks(2));
equ3 = 0 == det(A-B*K-k3*I);
subs(equ3, k3, Ks(3));
equ4 = 0 == det(A-B*K-k4*I);
subs(equ4, k4, Ks(4));

equ = [equ1, equ2, equ3, equ4];

S = solve(equ, K)