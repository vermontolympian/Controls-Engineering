%% simulation
clc; clear; close all

global robot dof jointTargetPos
robot = loadrobot('abbIrb120','DataFormat','column','Gravity',[0,0,-9.8]);  % load robot model, set data format to column, set gravity vector
dof = numel(homeConfiguration(robot));                                      % get robot degree of freedom
jointInitialPos = zeros(2*dof,1);                                           % define initial joint angles to be [0,0,0,0,0,0]
jointTargetPos = [pi/2, pi/3, pi/6, 2*pi/3, -pi/2, -pi/3]';                 % define desired joint angles. 

Tf = 1.0;                                                                   % simulation end time
tSpan = [0, Tf];                                                            % define simulation time span

tic;                                                                        % benchmarking
[T, X] = ode45(@(t,x)armODE(t,x),tSpan,jointInitialPos);                    % solve robot dynamical model dq=F(q,dq), robot state space is defined as X=[q, dq]
toc;

%% animation
figure()                                                                    % create new figure and set figure properties
set(gcf,'Visible','on');
show(robot, X(1,1:dof)');                                                   % show robot initial joint configuration from state space
view(60,10);                                                                % set 3D view (azimuth & elevation angle)
hold on
interval = round(0.01*length(X));                                           % set animation update interval (we have too many states)
for i = 1:interval:length(X)
    jointPos = X(i,1:dof);                                                  % get current joint positions from state space
    show(robot,jointPos','Frames','off','PreservePlot',false);              % show robot at current joint configuration
    title(sprintf('Frame = %d of %d', i, length(X)));                       % set figure title
    xlim([-0.8,0.8]); ylim([-0.8,0.8]); zlim([0,0.8]);                      % limit axis range
    drawnow                                                                 % force animation to update
end

%% utilities
function dx = armODE(~, x)
global jointTargetPos robot dof
    % ===== your code here =====
    % Hint: call jointPD function to calculate current joint torques needed
    % Hint: use forward function to calculate the joint acceleration
    % Hint: calculate what's dx
    
    torque = jointPD(jointTargetPos, zeros(dof,1), x);
    
    x_double_dot = forwardDynamics(robot, jointTargetPos, zeros(dof,1), torque);

    dx = [x((dof+1):end,1);x_double_dot];
end

function tau = jointPD(joint_target_pos,joint_target_vel,state)
global dof
    % ===== your code here =====
    kp = 40;
    kd = 25;
    tau = zeros(dof, 1);
    for i = 1:dof
        ep = joint_target_pos(i) - state(i,1);
        ev = joint_target_vel(i) - state((dof + i));
        
        u = (ep * kp) + (ev * kd) ;
        tau(i,1) = u;
    end
end
