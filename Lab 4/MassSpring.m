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


% Simulation
ts = 0.001;
t = 0:ts:60;
step(sys,t)
my_function_values = step(sys,t);
grid on;

[max_overshoot,Index] = max(my_function_values);

stepinfo(sys)

% Rise Time
% 10%
index_10 = min(find(my_function_values >= 0.1));
time_10 = index_10 * ts;
% 90%
index_90 = min(find(my_function_values >= 0.9));
time_90 = index_90 * ts;

Rise_Time = time_90 - time_10

% Peak Time
Peak_Time = Index * ts

% Max Overshoot
max_overshoot

% Settling Time (5%)

