clc; clear; close all

% system params
m = 0.1; % [kg] mass of cart
M = 1; % [kg] mass of pendulum
L = 1; % [m] Length of pendulum
g = 9.8; % [m/s^2] Gravity
u = 0; %control input Force

A = [0, 1, 0, 0;
     (M + m)*g/M*L, 0, 0, 0;
     0, 0, 0, 1;
     (-m*g)/M, 0, 0, 0];
B = [0; -1/(M*L); 0; 1/M];
C = [1 0 0 0;
    0 0 1 0];
D = [0 ; 0]; 

syms k1 k2 k3 k4 lambda

K = [k1, k2, k3, k4];

lambdas = [0, -2, -2+i, -2-i];


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

k = [k_1, k_2, k_3, k_4]

% calculate states
Ts = 0.01;
Tf = 10;
T = 0:Ts:Tf;
initial = [0,0.2,0,0.1]'; % initial condition
x = zeros(length(T),4);

for i = 1:length(T)
   % calculate states over time
   
   d = A*initial + B*u;
   
   x_new = initial + d*Ts;
   x(i,:) = x_new;
   initial = x_new;
   
   u = -k*x_new;
end


% Animation & Plots
figure()
grid on;
mass_w = 0.8;
mass_h = 0.5;
mass_d = 0.3;

% draw mass
[F, V] = stlread_mod('mass1.STL');
mass = patch('Faces',F,'Vertices',V(1:3,:)','Facecolor', [0.5 0 0.5], 'Edgecolor', 'none');


pendulum = line([x(1,3), x(1,3) + L*cos(x(1,1) + (pi/2))], [0, 0], [mass_d, mass_d + L*sin(x(1,1) + (pi/2))], 'LineWidth', 5);
ball = line([x(1,3) + L*cos(x(1,1) + (pi/2)), x(1,3) + L*cos(x(1,1) + (pi/2))], [0, 0], [mass_d + L*sin(x(1,1) + (pi/2)), mass_d + L*sin(x(1,1) + (pi/2))],'Marker', 'o', 'LineWidth', 8, 'Color', [1,0,0]);


view(3);
for i = 1:numel(T)
   T_O_n = eye(4); T_O_n(1,end) = x(i,3);
   V_new = T_O_n*V;
   mass.Vertices = V_new(1:3,:)';


   pendulum.XData = [mass_h + x(i,3), mass_h + x(i,3) + L*cos(x(i,1)+ (pi/2))]; 
   pendulum.ZData = [mass_d/2, mass_d/2 + L*sin(x(i,1) + (pi/2))];
   
   ball.XData = [mass_h + x(i,3) + L*cos(x(i,1)+ (pi/2)), mass_h + x(i,3) + L*cos(x(i,1)+ (pi/2))];
   ball.ZData = [mass_d/2 + L*sin(x(i,1) + (pi/2)), mass_d/2 + L*sin(x(i,1) + (pi/2))];
   
   title(['Iteration: ', num2str(i)]);
   xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
   axis equal
   xlim([-1.5, 2.5])
   pause(0.001);
end

figure()
plot(T,x, 'Linewidth', 5)
title('Inverted Pendulum Cart', 'Fontsize',20)
legend('Pendulum Angle', 'Pendulum Velocity','Cart Position', 'Cart Velocity', 'Fontsize', 20)