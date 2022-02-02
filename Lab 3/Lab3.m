%% spring mass simulation
clc; clear; close all

% system params
m = 5; % [kg]
k = 1; % [N/m]

% A, B, C, D
A = [0, 1; -k/m, 0];
B = [0; 1/m];

% calculate states
Ts = 0.02;
Tf = 40;
T = 0:Ts:Tf;
x_old = [0,0]'; % initial condition
x = zeros(length(T),2);
u = 1;

for i = 1:length(T)
   % calculate states over time
   % dx = Ax + Bu
   % x_new = x_old + (dx/dt)
   % x_old = x(i,:);
   
   d = A*x_old + B*u;
   
   x_new = x_old + d*Ts;
   x(i,:) = x_new;
   x_old = x_new;
end


%% Animation & Plots
figure()
grid on;
mass_w = 0.8;
mass_h = 0.5;
mass_d = 0.3;

% draw mass
[F, V] = stlread_mod('mass1.STL');
% draw mass
mass = patch('Faces',F,'Vertices',V(1:3,:)','Facecolor', [0.5 0 0.5], 'Edgecolor', 'none');
% draw simplified spring
spring = line([0, x(1,1)], [0,0], [mass_d/2, mass_d/2], 'Linewidth', 5);   % draw spring
view(3);
for i = 1:numel(T)
   T_O_n = eye(4); T_O_n(1,end) = x(i,1);
   V_new = T_O_n*V;
   mass.Vertices = V_new(1:3,:)';
   spring.XData = [0, x(i,1)];
   
   title(['Iteration: ', num2str(i)]);
   xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
   axis equal
   xlim([-0.5, 3])
   pause(0.001);
end