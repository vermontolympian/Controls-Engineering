clc; clear; close all;
num=[0 0 0 10];
den=[1 6 8 10];
%printsys(num,den)
t = 0:0.01:10;
[y,r,t] = step(num, den, t);
plot(t,y)
grid on
title('Unit Step Response')
xlabel('t (seconds)')
ylabel('output')
r1 = 1; while y(r1) < 0.1, r1 = r1 + 1; end
r2 = 1; while y(r2) < 0.9, r2 = r2 + 1; end
rise_time = (r2-r1)*0.01
[ymax, tp] = max(y);
peak_time = (tp-1)*0.01
max_overshoot = ymax - 1
s = 1001; while y(s) > 0.98 & y(s) < 1.02; s = s - 1; end
settling_time = (s-1) * 0.01