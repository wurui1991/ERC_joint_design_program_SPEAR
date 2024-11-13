% =========================================================================
% ERC design step 3: cam 3D modeller
%
% Author: Rui Wu (rui.wu@usys.ethz.ch)
%         Stefano Mintchev (stefano.mintchev@usys.ethz.ch)
% Environmental Robotics Lab, ETH Zurich, 2024
%
% Funded by the Horizon Europe project in AI & robotics:
% "SPEAR: Spatial Perception & Embodied Autonomy Research"
% =========================================================================

%% !! NOTE: run ERCdesigner.m first before you run this program

%% INPUT

Rtooth = 2; % tooth diameter (mm)
gap = 4; % gap between tooth (mm); default: 2*Rtooth
H = 30; % overall thickness [mm]
Larm = 20; % depth of arm (e.g., the CFRP tube) mountng hole
L = 25; % length of ERC from pivot to the mounting hole
Rarm = 8; % inner radius of mountng hole
R = 9; % outer radius around mounting hole
Rpivot = 1.5; % radius of spring pivot hole

%% Generate cam contour with equally spaced points along the curve
XY_mm = XY*1000; % unit covertion to [mm]
nPoints = size(XY_mm,2)*2;
% Calculate the cumulative distance (arc length) along the curve
dx = diff(XY_mm(1,:));
dy = diff(XY_mm(2,:));
distances = sqrt(dx.^2 + dy.^2);
cumDist = [0, cumsum(distances)];
space = max(cumDist)/(nPoints-1); % space between the points
% Generate the evenly spaced values of the cumulative distance
evenDist = linspace(0, cumDist(end), nPoints);
% Interpolate to find x and y values at these evenly spaced distances
x_interp = interp1(cumDist, XY_mm(1,:), evenDist, 'linear');
y_interp = interp1(cumDist, XY_mm(2,:), evenDist, 'linear');
% Combine the interpolated x and y values
XY_mm_interp = [x_interp; y_interp]; % cam contour with equally spaced points along the curve

%% tooth modelling
gapindex=round(gap/space); % tooth gap in terms of XY_mm_interp index
%%% perpendicular vectors from cam contour (XY_mm_interp)
px = -diff(XY_mm_interp(2, :));
py = diff(XY_mm_interp(1, :));
Plength = sqrt(px.^2+py.^2);
Pvectors = [px; py];
Pvectors = Pvectors ./ Plength;
Pvectors = [Pvectors Pvectors(:,end)];
%%% offset cam contour (XY_mm_interp) to align tooth (spherical tooth offset from centre by Rtooth/3)
XY_mm_in = XY_mm_interp+Pvectors*Rtooth/3; % for convex tooth
XY_mm_out = XY_mm_interp-Pvectors*Rtooth/3; % for concave tooth

%% 3D modelling
offset = XY_mm(1,1); % X offset of head rectangle (from pivot to cam profile)
% head rectangle
headz=[H/2; -1*H/2; -1*H/2; H/2; H/2];
headz = [H/2; -1*H/2; -1*H/2; H/2; H/2];
heady = [XY_mm(2,end); XY_mm(2,end); XY_mm(2,1); XY_mm(2,1); XY_mm(2,end)];
headx = [XY_mm(1,end); XY_mm(1,end); XY_mm(1,1); XY_mm(1,1); XY_mm(1,end)];
%%% tooth locations
% convex tooth
convextooth1 = [XY_mm_in(1,1:2*gapindex:end);XY_mm_in(2,1:2*gapindex:end);...
    (H/2-Rtooth-1)*ones(size(XY_mm_in(2,1:2*gapindex:end)))];% convex tooth row 1
convextooth2 = [XY_mm_in(1,gapindex:2*gapindex:end);XY_mm_in(2,gapindex:2*gapindex:end);...
    (H/2-Rtooth-gap-1)*ones(size(XY_mm_in(2,gapindex:2*gapindex:end)))];% convex tooth row 2
convextoothall = [convextooth1 convextooth2];
convextoothall = [convextoothall convextoothall.*[1 1 -1]']; % convex tooth locations
% concave tooth
concavetooth1 = [XY_mm_out(1,1:2*gapindex:end);-1*XY_mm_out(2,1:2*gapindex:end);...
    (H/2-Rtooth-1)*ones(size(XY_mm_out(2,1:2*gapindex:end)))];% concave tooth row 1
concavetooth2 = [XY_mm_out(1,gapindex:2*gapindex:end);-1*XY_mm_out(2,gapindex:2*gapindex:end);...
    (H/2-Rtooth-gap-1)*ones(size(XY_mm_out(2,gapindex:2*gapindex:end)))];% concave tooth row 2
concavetoothall = [concavetooth1 concavetooth2];
concavetoothall = [concavetoothall.*[1 -1 1]' concavetoothall.*[1 -1 -1]']; % concave tooth locations

%% generating AutoCAD .scr script
% ERC with convex tooth
toothall = convextoothall; % initialise with convex tooth
filename = 'convex.scr';
ERCscr % generate the AutoCAD .scr script

% ERC with concave tooth
toothall = concavetoothall; % initialise with concave tooth
filename = 'concave.scr';
Rtooth = Rtooth+0.05; % adding a small tolerance (0.05 mm) to tooth radius
ERCscr % generate the AutoCAD .scr script