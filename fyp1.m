 function varargout = fyp1(varargin)
% FYP1 MATLAB code for fyp1.fig
%      FYP1, by itself, creates a new FYP1 or raises the existing
%      singleton*.
%
%      H = FYP1 returns the handle to a new FYP1 or the handle to
%      the existing singleton*.
%
%      FYP1('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in FYP1.M with the given input arguments.
%
%      FYP1('Property','Value',...) creates a new FYP1 or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before fyp1_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to fyp1_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help fyp1

% Last Modified by GUIDE v2.5 04-Dec-2020 15:24:47

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @fyp1_OpeningFcn, ...
                   'gui_OutputFcn',  @fyp1_OutputFcn, ...
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
end


% --- Executes just before fyp1 is made visible.
function fyp1_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to fyp1 (see VARARGIN)

% Choose default command line output for fyp1
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes fyp1 wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end


% --- Outputs from this function are returned to the command line.
function varargout = fyp1_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%clear all
%close all
clc
% sim=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
% sim.simxFinish(-1); % just in case, close all opened connections
% clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);

Th_1 = str2double(handles.Theta1.String)*pi/180;
Th_2 = (str2double(handles.Theta2.String)-0)*pi/180;
Th_3 = (str2double(handles.Theta3.String)+90)*pi/180;
Th_4 = (str2double(handles.Theta4.String)-0)*pi/180;
Th_5 = (str2double(handles.Theta5.String)+0)*pi/180;
Th_6 = str2double(handles.Theta6.String)*pi/180;

% 1 degree = 0.0174532925 radians
% 1 degree * (pi / 180) = 0.0174532925 radians

%% Figure GUI Robot

% % Link length
% L_1 = 0; %20 0
% L_2 = 60; %50 80
% L_3 = 40; %40 70
% L_4 = 30;
% L_5 = 20;
% L_6 = 20;

% L_1 = 350; %20 0
% L_2 = 850; %50 80
% L_3 = 20; %40 70
% L_4 = 410;
% L_5 = 410;
% L_6 = 20;

%% cuba baru

L_1 = 0; %20 0
L_2 = 50; %50 80
L_3 = 0; %40 70
L_4 = 0;
L_5 = 0;
L_6 = 0;

%Link([theta, d, a, alpha])
L(1) = Link([0 25 L_1 pi/2]);
L(2) = Link([0 0 L_2 0]);
L(3) = Link([0 0 L_3 pi/2]);
L(4) = Link([0 50 L_4 -pi/2]);
L(5) = Link([0 0 L_5 pi/2]);
L(6) = Link([0 0 L_6 0]);

% %Link([theta, d, a, alpha])
% L(1) = Link([0 40 L_1 0]);
% L(2) = Link([0 0 L_2 -pi/2]);
% L(3) = Link([0 40 L_3 -pi/2]);
% L(4) = Link([0 0 L_4 -pi/2]);
% L(5) = Link([0 0 L_5 0]);
% L(6) = Link([0 -40 L_6 -pi/2]);

%Link([theta, d, a, alpha])
% L(1) = Link([0 0 L_1 0]);
% L(2) = Link([0 0 L_2 pi/2]);
% L(3) = Link([0 0 L_3 0]);
% L(4) = Link([0 965.53216 L_4 pi/2]);
% L(5) = Link([0 0 L_5 -pi/2]);
% L(6) = Link([0 165.53216 L_6 pi/2]);

%% lama

% %Link([theta, d, a, alpha])
% L(1) = Link([0 60 L_1 pi/2]);
% L(2) = Link([0 0 L_2 0]);
% L(3) = Link([0 0 L_3 0]);
% L(4) = Link([0 0 L_4 0]);
% L(5) = Link([0 0 L_5 0]);
% L(6) = Link([0 0 L_6 0]);

% L(1) = Link([0 815 L_1 pi/2]);
% L(2) = Link([0 200 L_2 0]);
% L(3) = Link([0 -200 L_3 0]);
% L(4) = Link([0 145 L_4 0]);
% L(5) = Link([0 0 L_5 0]);
% L(6) = Link([0 0 L_6 0]);

% L(1) = Link([0 0 L_1 -90]);
% L(2) = Link([0 0 L_2 0]);
% L(3) = Link([0 0 L_3 -90]);
% L(4) = Link([0 670 L_4 90]);
% L(5) = Link([0 0 L_5 -90]);
% L(6) = Link([0 115 L_6 0]);

% Kuka_2 = (str2double(handles.Theta2.String) + 135)*pi/180
% Kuka_3 = (str2double(handles.Theta3.String) - 135)*pi/180

Robot = SerialLink(L);
Robot.name = 'Robots';
Robot.plot([Th_1 Th_2 Th_3 Th_4 Th_5 Th_6]);


%% CoppeliaSim Robot

% % i = Th_1 
% % o = Th_2 
% p = ((Th_3 / (pi/180)) - 90) * pi/180
% 
% % create some joints pos
% joint_pos=[Th_1,Th_2,p,Th_4,Th_5,Th_6];
% %joint_pos1=[-2*pi/3,0,0,0,pi/4,0];
% %joint_pos2=[0,-pi/4,pi/2,0,0,0];
% %joint_pos3=[0,0,0,0,0,0];
% 
% %joints handles
% h=[0,0,0,0,0,0]
%   [r,h(1)]=sim.simxGetObjectHandle(clientID,'rotary_head',sim.simx_opmode_blocking);
%   [r,h(2)]=sim.simxGetObjectHandle(clientID,'lower_arm',sim.simx_opmode_blocking);
%   [r,h(3)]=sim.simxGetObjectHandle(clientID,'upper_arm',sim.simx_opmode_blocking);
%   [r,h(4)]=sim.simxGetObjectHandle(clientID,'forearm_twisting',sim.simx_opmode_blocking);
%   [r,h(5)]=sim.simxGetObjectHandle(clientID,'wrist',sim.simx_opmode_blocking);
%   [r,h(6)]=sim.simxGetObjectHandle(clientID,'axis6',sim.simx_opmode_blocking);
%   
% for i=1:6
% sim.simxSetJointTargetPosition(clientID,h(i),joint_pos(i),sim.simx_opmode_streaming);
% end


%% DH Convention to prove position of end effector

% syms L_1 L_2 L_3 L_4 L_5 L_6
% syms theta1 theta2 theta3 theta4 theta5 theta6

% theta1 = str2double(handles.Theta1.String);
% theta2 = (str2double(handles.Theta2.String)) + 135;
% theta3 = (str2double(handles.Theta3.String)) - 135;
% theta4 = str2double(handles.Theta4.String);
% theta5 = str2double(handles.Theta5.String);
% theta6 = str2double(handles.Theta6.String);

% theta1 = str2double(handles.Theta1.String)*pi/180;
% theta2 = (str2double(handles.Theta2.String)-90)*pi/180;
% theta3 = (str2double(handles.Theta3.String)+0)*pi/180;
% theta4 = (str2double(handles.Theta4.String)-90)*pi/180;
% theta = (str2double(handles.Theta5.String)+90)*pi/180;
% theta4 = str2double(handles.Theta6.String)*pi/180;

theta1 = str2double(handles.Theta1.String);
theta2 = str2double(handles.Theta2.String);
theta3 = str2double(handles.Theta3.String)+90;
theta4 = str2double(handles.Theta4.String);
theta5 = str2double(handles.Theta5.String);
theta6 = str2double(handles.Theta6.String);



% DH Parameters
a1 = L_1; alpha1 = 180/2; d1 = 25; theta1 = theta1;   % Link 1
a2 = L_2; alpha2 = 0; d2 = 0; theta2 = theta2;   % Link 2
a3 = L_3; alpha3 = 180/2; d3 = 0; theta3 = theta3;   % Link 3
a4 = L_4; alpha4 = -180/2; d4 = 50; theta4 = theta4;   % Link 4
a5 = L_5; alpha5 = 180/2; d5 = 0; theta5 = theta5;   % Link 5
a6 = L_6; alpha6 = 0; d6 = 0; theta6 = theta6;   % Link 6

% Insert parameters in DH function
H0_1 = DHmatrix(a1, alpha1, d1, theta1)
H1_2 = DHmatrix(a2, alpha2, d2, theta2)
H2_3 = DHmatrix(a3, alpha3, d3, theta3)
H3_4 = DHmatrix(a4, alpha4, d4, theta4)
H4_5 = DHmatrix(a5, alpha5, d5, theta5)
H5_6 = DHmatrix(a6, alpha6, d6, theta6)

H0_6 = H0_1 * (H1_2 * (H2_3 * (H3_4 * (H4_5 * H5_6))))

% R13 = cos(theta_1)*(cos(theta_2)*(cos(theta_5)*sin(theta_3) + cos(theta_3)*cos(theta_4)*sin(theta_5)) + sin(theta_2)*(cos(theta_3)*cos(theta_5) - cos(theta_4)*sin(theta_3)*sin(theta_5))) + sin(theta_1)*sin(theta_4)*sin(theta_5);
% R23 = sin(theta_1)*(cos(theta_2)*(cos(theta_5)*sin(theta_3) + cos(theta_3)*cos(theta_4)*sin(theta_5)) + sin(theta_2)*(cos(theta_3)*cos(theta_5) - cos(theta_4)*sin(theta_3)*sin(theta_5))) - cos(theta_1)*sin(theta_4)*sin(theta_5);
% R33 = sin(theta_2)*(cos(theta_5)*sin(theta_3) + cos(theta_3)*cos(theta_4)*sin(theta_5)) - cos(theta_2)*(cos(theta_3)*cos(theta_5) - cos(theta_4)*sin(theta_3)*sin(theta_5));

%simplify(H0_6)

positionx = H0_6 (1,4);
positiony = H0_6 (2,4);
positionz = H0_6 (3,4);

set(handles.PosX,'String',positionx);
set(handles.PosY,'String',positiony);
set(handles.PosZ,'String',positionz);

% ttt = asind(((positionx)^2 + (positiony)^2 + (positionz)^2 - (d4^2) - (d2^2)) / a2*d4*2)



%% pdf
% k = (positionx + positiony + positionz - d4 - a2) / (2*a2*d4);
% 
% v = (1 + sqrt(1-(k^2))) / k;
% w = (1 - sqrt(1-(k^2))) / k;
% 
% tii3 = atan2(v,w)
% 
% tii4 = atan2(v,w) / (pi/180)

%% tenet theta3
k2 = ((positionx)^2 + (positiony)^2 + (positionz)^2 - (d3^2)  - (d4^2) - (a2^2) - (a3^2)) / (2*a2);

e = (-d4 + sqrt((a3^2) + (d4^2) + (k2^2))) / (k2+a3);

r = (-d4 - sqrt((a3^2) + (d4^2) + (k2^2))) / (k2+a3);

tita3 = atan2(r,e);

tita4 = atan2(r,e) / (pi/180);

%% tenet theta2


end



function Theta1_Callback(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta1 as text
%        str2double(get(hObject,'String')) returns contents of Theta1 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function Theta2_Callback(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta2 as text
%        str2double(get(hObject,'String')) returns contents of Theta2 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function Theta3_Callback(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta3 as text
%        str2double(get(hObject,'String')) returns contents of Theta3 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function Theta4_Callback(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta4 as text
%        str2double(get(hObject,'String')) returns contents of Theta4 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function Theta5_Callback(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta5 as text
%        str2double(get(hObject,'String')) returns contents of Theta5 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function Theta6_Callback(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Theta6 as text
%        str2double(get(hObject,'String')) returns contents of Theta6 as a double
end


% --- Executes during object creation, after setting all properties.
function Theta6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to Theta6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end



function PosX_Callback(hObject, eventdata, handles)
% hObject    handle to PosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PosX as text
%        str2double(get(hObject,'String')) returns contents of PosX as a double
end


% --- Executes during object creation, after setting all properties.
function PosX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PosX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function PosY_Callback(hObject, eventdata, handles)
% hObject    handle to PosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PosY as text
%        str2double(get(hObject,'String')) returns contents of PosY as a double
end


% --- Executes during object creation, after setting all properties.
function PosY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PosY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end


function PosZ_Callback(hObject, eventdata, handles)
% hObject    handle to PosZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of PosZ as text
%        str2double(get(hObject,'String')) returns contents of PosZ as a double
end


% --- Executes during object creation, after setting all properties.
function PosZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to PosZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
end
