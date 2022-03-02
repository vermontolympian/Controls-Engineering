function varargout = Lab7(varargin)

global robot dof X X_des
robot = loadrobot('abbIrb120','DataFormat','column','Gravity',[0,0,-9.8]);  % load robot model
dof = numel(homeConfiguration(robot));          % degree of freedom = 6


% LAB7 MATLAB code for Lab7.fig
%      LAB7, by itself, creates a new LAB7 or raises the existing
%      singleton*.
%
%      H = LAB7 returns the handle to a new LAB7 or the handle to
%      the existing singleton*.
%
%      LAB7('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in LAB7.M with the given input arguments.
%
%      LAB7('Property','Value',...) creates a new LAB7 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Lab7_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Lab7_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Lab7

% Last Modified by GUIDE v2.5 02-Mar-2022 11:00:03

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Lab7_OpeningFcn, ...
                   'gui_OutputFcn',  @Lab7_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Lab7 is made visible.
function Lab7_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Lab7 (see VARARGIN)

% Choose default command line output for Lab7
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Lab7 wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Lab7_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes during object creation, after setting all properties.
function robot_display_CreateFcn(hObject, eventdata, handles)
% hObject    handle to robot_display (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate robot_display


% --- Executes on slider movement.
function joint1_slider_Callback(hObject, eventdata, handles)
% hObject    handle to joint1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global X_des
X_des(1) = get(hObject, 'Value')
set(handles.edit1, 'String', num2str(X_des(1)))


% --- Executes during object creation, after setting all properties.
function joint1_slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to joint1_slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called




function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in Initialize.
function Initialize_Callback(hObject, eventdata, handles)
% hObject    handle to Initialize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global X X_des robot dof
robot = loadrobot('abbIrb120','DataFormat','column','Gravity',[0,0,-9.8]);  % load robot model
dof = numel(homeConfiguration(robot));          % degree of freedom = 6
X = zeros(2*dof, 1);
X_des = zeros(2*dof, 1);

show(robot, zeros(dof,1), 'Parent', handles.robot_display)
view(handles.robot_display, 120, 30)
xlim(handles.robot_display, [-.5, .5]);
ylim(handles.robot_display, [-.5, .5]);
zlim(handles.robot_display, [0, 1]);


% --- Executes on button press in go.
function go_Callback(hObject, eventdata, handles)
% hObject    handle to go (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global robot dof X X_des
tSpan = [0 0.2];
disp('initial')
disp(X(1:dof));
disp('desired')
disp(X_des(1:dof))
tic;
[T_rec, X_rec] = ode23(@(t,x)armODE(t,x),tSpan,deg2rad(X),odeset('AbsTol',1e-2));  
% for faster execution, we use ode23 and allow lower precision
toc;
X = X_des;
% animation
interval = round(0.01*length(T_rec));
for i = 1:interval:length(T_rec)
    view(handles.robot_display,120,30);
%     % show current joint position in GUI
%     set(handles.joint1_current,'String',num2str(rad2deg(X_rec(i,1)),'%.2f'))
%     set(handles.joint2_current,'String',num2str(rad2deg(X_rec(i,2)),'%.2f'))
%     set(handles.joint3_current,'String',num2str(rad2deg(X_rec(i,3)),'%.2f'))
%     set(handles.joint4_current,'String',num2str(rad2deg(X_rec(i,4)),'%.2f'))
%     set(handles.joint5_current,'String',num2str(rad2deg(X_rec(i,5)),'%.2f'))
%     set(handles.joint6_current,'String',num2str(rad2deg(X_rec(i,6)),'%.2f'))
    % show robot configuration in GUI
    show(robot,X_rec(i,1:dof)','Parent',handles.robot_display,'PreservePlot',false);
    xlim(handles.robot_display,[-.5, .5]); 
    ylim(handles.robot_display,[-.5, .5]); 
    zlim(handles.robot_display,[0.0, 1.0]);
    title(sprintf('Frame = %d of %d', i, length(T_rec)));
    drawnow;
end



function dx = armODE(~, x)
global robot dof X_des
    tau = jointPD(deg2rad(X_des(1:dof)), zeros(dof,1), x);
    dx = zeros(dof*2,1);
    dx(1:dof) = x(dof+1:end);
    dx(dof+1:end) = forwardDynamics(robot,x(1:dof),[],tau,[]);


function tau = jointPD(joint_target_pos,joint_target_vel,x)
    % ===== your code here =====
    % **Hint**: implement the same thing you did in lab5
    % **Hint**: you may consider different Kp and Kd for different joint
    % to achieve better tracking performance
    
    kp = [300, 300, 300, 300, 300, 300];
    kd = [30, 30, 30, 30, 30, 30];
    tau = zeros(size(joint_target_pos,1), 1);
    for i = 1:size(joint_target_pos,1)
        ep = joint_target_pos(i) - x(i,1);
        ev = joint_target_vel(i) - x(6+i, 1);
        
        u = (ep * kp(i)) + (ev * kd(i));
        tau(i,1) = u;
    end
    
    
   


% --- Executes on button press in home.
function home_Callback(hObject, eventdata, handles)
% hObject    handle to home (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global X_des dof
X_des = zeros(2*dof,1);
set(handles.joint1_slider, 'Value', 0);
set(handles.edit1,'String', num2str(0));
