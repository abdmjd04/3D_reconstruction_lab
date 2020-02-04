%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Visual Perception Lab-I                                                  %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clc; close all; clear all;

[verts, faces, cindex] = teapotGeometry;
figure; 
p = patch('Faces',faces,'Vertices',verts,'FaceVertexCData',cindex,'FaceColor','interp')
X =verts
view(-151,30)     % change the orientation
axis equal on    % make the axes equal and invisible
p.FaceAlpha = 0.3; 
p.FaceColor = 'none'; 
p.FaceColor = 'none';    % turn off the colors
p.FaceAlpha = 1;           % remove the transparency
p.FaceColor = 'interp';    % set the face colors to be interpolated
p.LineStyle = 'none';      % remove the lines
colormap(copper)           % change the colormap
l = light('Position',[-0.4 0.2 0.9],'Style','infinite')
lighting gouraud
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-5,5]);
ylim([-5,5]);
zlim([-5,5]);
title('Simulated 3D scene ');

%% Simulating First camera 
%load camera parameters
load('cameraparams.mat')
disp('Real Intrinsics obtained using Calibration Toolbox')
K1 = cameraParams.IntrinsicMatrix'
normalizeCam1 = [eye(3) zeros(3,1)]; %normalsised camera matrix
R1 = eye(3);
t1 = [ 0 0 0]'; % camera 1 at origin

% %Camera MatriX
P1 = K1 * normalizeCam1 * [R1 t1; zeros(1,3) 1];

%% For first camera
% projections of scene 
x1 = projection(P1, X');

% construct camera structure
C1 = camstruct('f', 947);

% convert image pts to a plane
pt1 = imagept2plane(C1, x1(1:2,:), [0 0 947], [0 0 1]);
% to plot the points on image plane
figure
plot3(pt1(1,:),pt1(2,:), pt1(3,:),'*','MarkerSize',10, 'LineWidth', 1,'Color','b');
title('projection with respect to first camera');

%% Implementing DLT

[P, rperr] = CalibDLT(x1, X)

%Decompose P matrix
[K,R,t] = DecompPMat(P)

disp('*****************************************************')
disp('DLT Computed intrinsic Parameters')
K

%% Simulating Second Camera 

K2 =K1;
normCam2 = [eye(3) zeros(3,1)]; %normalsised camera matrix
R2 = eye(3);
txx = 300; % trsnalatoin along x
t2 = [ -txx 0 0]'; %to trsnalate camera by 400 Extrinsic parameters

%Camera Matric of camera2
P2 = K2 * normCam2 * [R2 t2; zeros(1,3) 1];

%% For second camera 

%projections of scene 
x2 = projection(P2, X');

C2 = camstruct('f', 947, 'P', [txx;0;0]); % to construct camera structure to be used by imagept2plane function
% convert image points to a  a plane
pt2 = imagept2plane(C2, x2(1:2,:), [0 0 947], [0 0 1]);
figure
%%% uncomment to see projection scene
plot3(pt2(1,:),pt2(2,:), pt2(3,:),'.','MarkerSize',10, 'LineWidth', 1,'Color','b');
title('projection with respect to second camera');

%% 3D DLT Reconstruction
reconX = Reconstruction(P1, P2, x1, x2)
% recon2 = reconX'
% recon2 = recon2(:,1:end-1)
% %pcshow(recon2)
% %view(-151,30)     % change the orientation
% %axis equal on
figure
plot3(reconX(1,:), reconX(2,:), reconX(3,:),'.','MarkerSize',10, 'LineWidth', 1,'Color','b')
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-3,4]);
ylim([-3,4]);
zlim([-3,4]);
title('3D Reconstruction using DLT');
