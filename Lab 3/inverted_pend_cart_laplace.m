% Transfer Function

M = 1; % kg
m = 0.1; % kg
L = 1; % m
g = 9.8; %m/s squared
I = 0.05; % moment of inertia %%this number is wrong!!!
q = (M+m)*(I+m*L^2)-(m*L)^2;
s = tf('s'); 

P_cart = (((I+m*L^2)/q)*s^2 - (m*g*L/q))/(s^4 + ((I + m*L^2))*s^3/q - ((M + m)*m*g*L)*s^2/q - m*g*L*s/q);

P_pend = (m*L*s/q)/(s^3 + ((I + m*L^2))*s^2/q - ((M + m)*m*g*L)*s/q - m*g*L/q);

sys_tf = [P_cart ; P_pend];

P = pole(sys_tf)

inputs = {'u'};
outputs = {'x'; 'theta'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf
