%% Lab6 code template
clc; clear; close all

global robot p_joints
robot = loadrobot('abbIrb120','DataFormat','column','Gravity',[0,0,-9.8]);  % load robot model
dof = numel(homeConfiguration(robot));          % degree of freedom = 6
jointInitialPos = zeros(dof,1);
jointInitialVel = zeros(dof,1);
X0 = [jointInitialPos; jointInitialVel];        % initial condition: 0 position & 0 velocity

% ========== define task-space trajectory ==========
Tf = 1.0;
tSpan = [0, Tf];
wayPoints = [0.3740, 0.18, 0.4, 0.4, 0.18; ...              % define task-space waypoints [X; Y; Z]
             0, -0.2, -0.2, 0.2, 0.2; ...
             0.63, 0.5, 0.5, 0.5, 0.5];
timePoints = linspace(0, Tf, length(wayPoints));            % define time points reaching each waypoint
tSamples = 0:0.01:1;                                        % sampling trajectory at 0.01 sec
[q,dq,~,pp] = bsplinepolytraj(wayPoints, tSpan, tSamples);  % generate task-space trajectory
% plot3(q(1,:),q(2,:),q(3,:),'.b'); grid on         % uncomment to visualize task-space trajectory
% ==================================================

% ========== solve for joint-space trajectory ==========
ik = inverseKinematics('RigidBodyTree',robot);  % create inverse kinematic solver
ik_sol_weights = [0.1,0.1,0.1,5.0,5.0,5.0]';    % define solution tolerence
target_joint_pos_rec = [];
for i = 1:length(q)
    eef_pose = eye(4);                          % set desired end-effector pose
    eef_pose(1,end) = q(1,i);
    eef_pose(2,end) = q(2,i);
    eef_pose(3,end) = q(3,i);
    [configSol,~] = ik(robot.BodyNames{end},eef_pose,ik_sol_weights,robot.homeConfiguration);   % solve inverse kinematics
    target_joint_pos_rec = [target_joint_pos_rec, configSol];
end
p_joints = [];
for i = 1:dof
    p = polyfit(tSamples,target_joint_pos_rec(i,:),7);  % use 7th order polynomial to generate joint-space trajectory
    p_joints = [p_joints;p];                            % record polynomial coefficients (each joint has different trajectory)
end
% ======================================================

tic;
[T, X] = ode23(@(t,x)armODE(t,x),tSpan,X0,odeset('AbsTol',1e-2));   % for faster execution, we use ode23 and allow lower precision
toc;    % may still run for several minutes, depending on how much computation power you have

%% animation
figure('Position',[1920/3,1080/3,1000,460])
set(gcf,'Visible','on');
show(robot, X(1,1:dof)');
view(60,10);
hold on
plot3(q(1,:),q(2,:),q(3,:),'.r')                                    % plot desired end-effector trajectory
interval = round(0.01*length(X));
for i = 1:interval:length(X)
    eef_pose = getTransform(robot,X(i,1:dof)','tool0','base');      % get end-effector pose as a transformation matrix
    show(robot,X(i,1:dof)','PreservePlot',false);                   % plot robot configuration
    plot3(eef_pose(1,end),eef_pose(2,end),eef_pose(3,end),'.b')     % plot actual end-effector trajectory
    title(sprintf('Frame = %d of %d', i, length(X)));
    xlim([-0.8,0.8]); ylim([-0.8,0.8]); zlim([0,0.8]);
    drawnow
end

%% plot tracking error
figure('Position',[1920/3,1080/3,1100,500])
for plt = 1:dof
    subplot(2,3,plt);
    track_err = polyval(p_joints(plt,:),T)-X(:,plt);    % calculate tracking error
    plot(T,X(:,plt),'LineWidth',1)                      % plot actual joint angles
    hold on
    plot(T,polyval(p_joints(plt,:),T))                  % plot reference joint angles
    grid on
    xlabel('time [sec]');
    ylabel('joint angle [rad]');
    legend(['q',num2str(plt)],'ref')
    title(['q',num2str(plt),' avg tracking err [rad]: ', num2str(mean(track_err))]);
end

%% utilities
function [angle, delta_angle] = getJointDesired(t)
    global p_joints
    % ===== your code here =====
    % **Hint**: use 'polyval' to calculate the desired joint angles at 
    % current time 't' with the polynomial coefficients 'p_joints'
    % **Hint**: use 'polyder' and 'polyval' to calculate the desired joint 
    % velocies at current time 't' with the polynomial coefficients 'p_joints'
end

function dx = armODE(t, x)
global robot
    % ===== your code here =====
    % **Hint**: first call 'getJointDesired' function to calculate the
    % desired joint positions and velocities at current time 't'
    % **Hint**: the rest part is the same as lab5
end

function tau = jointPD(joint_target_pos,joint_target_vel,x)
    % ===== your code here =====
    % **Hint**: implement the same thing you did in lab5
    % **Hint**: you may consider different Kp and Kd for different joint
    % to achieve better tracking performance
end
