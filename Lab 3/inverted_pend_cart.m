% spring mass simulation
clc; clear; close all

% system params
m = 0.1; % [kg]
M = 1; % [kg]
L = 1; % [m]
g = 9.8; % [m/s^2]

% A, B, C, D
A = [0, 1, 0, 0; (M+m)*g/(M*L), 0, 0, 0; 0, 0, 0, 1; -(m*g)/M, 0, 0, 0];
B = [0; -1/(M*L); 0; 1/M];

% calculate states
Ts = 0.02;
Tf = 40;
T = 0:Ts:Tf;
initial = [0,0.5,0,0]'; % initial condition
x = zeros(length(T),4);
u = 0;

for i = 1:length(T)
   % calculate states over time
   
   d = A*initial + B*u;
   
   x_new = initial + d*Ts;
   x(i,:) = x_new;
   initial = x_new;
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
   xlim([-6, 6])
   pause(0.001);
end