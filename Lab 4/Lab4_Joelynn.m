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

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'x' ; 'phi'};

sys_ss = ss(A,B,C,D, 'statename', states, 'inputname', inputs, 'outputname', outputs)

sys_tf = tf(sys_ss)

poles = eig(A)1341449