clc;
clear;
close all

A = eye(3);
B = eye(3);
C = [1;4;7];
D = (A+B)*C;

t = 0:1/20:10;
y = sin(t);
plot(t,y);
xlabel('Seconds (t)');
ylabel('sin(t)');
title('Plot of Sine Function')

s=tf('s');
myFunction = s^4+3*s^3-15*s^2-2*s+9;

roots = zero(myFunction);