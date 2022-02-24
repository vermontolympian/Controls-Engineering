clc; clear; close all

m1 = 1;
m2 = 2;
k1 = 5;
k2 = 1;
b = 2;

num = [k1*b, k1*k2];
den = [m1*m2, (m1 + m2)*b, (k1*m2 + (m1 + m2)*k2), k1*b, k1*k2];

sys = tf(num, den)

bode(sys)