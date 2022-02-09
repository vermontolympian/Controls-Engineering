% spring mass simulation
clc; clear; close all

% System Params
m1 = 1;
m2 = 2;
k1 = 5;
k2 = 5;
b = 10;


s=tf('s');

syms s

numerator = [k1*b, k1*k2];
denominator = [m1*m2, b*(m1+m2), (k1*m2 + (m1 +m2)*k2), k1*b, k1*k2];
(m1*m2*2^4) + b*s^3*(m1 + m2) + (k1*m2 + (m1 + m2)*k2)*s^2 + k1*b*s + k1*k2;

Y = (k1*((b*s)+k2))/((m1*m2*2^4) + b*s^3*(m1 + m2) + (k1*m2 + (m1 + m2)*k2)*s^2 + k1*b*s + k1*k2);

sys = tf(numerator, denominator)

t = 20:0.2:120;
step(sys,t)

% sys_tf