% =========================================================================
% ERC design step 4: AutoCad .scr code generator
%
% Author: Rui Wu (rui.wu@usys.ethz.ch)
%         Stefano Mintchev (stefano.mintchev@usys.ethz.ch)
% Environmental Robotics Lab, ETH Zurich, 2025
%
% Funded by the Horizon Europe project in AI & robotics:
% "SPEAR: Spatial Perception & Embodied Autonomy Research"
% =========================================================================

%% !! NOTE: this program should be used with ERCmodeller.m

% Continue writing to the same file, appending new commands
fileID = fopen(filename, 'w'); % Open the file in write mode
if fileID == -1
    error('Failed to open file for appending.');
end

% Cam; Draw polyline for [XY_mm(1,:)'; XY_mm(2,:)'; H/2*ones(size(XY_mm(1,:)'))] and extrude it _Z by "H"
fprintf(fileID, '_3dpoly\n');
for i = 1:size(XY_mm, 2)
    fprintf(fileID, '%f,%f,%f\n', XY_mm(1, i), XY_mm(2, i), H / 2);
end
fprintf(fileID, 'c\n'); % Close the polyline
fprintf(fileID, '_extrude\n_last\n\n%d\n', -1*H/3);

fprintf(fileID, '_copy\n');  % Initiate move command
fprintf(fileID, '_last\n');  % Select the last created object
fprintf(fileID, '\n');  % Confirm selection
fprintf(fileID, '0,0,0\n');  % Specify base point for move
fprintf(fileID, '0,0,%f\n', -2*H/3);  % Specify second point (destination)

% Spring pivot hole; Draw a circle at 0,0,H/2, with radius Rpivot, and extrude it -Z by H
fprintf(fileID, '_circle 0,0,%f %f\n', H / 2, Rpivot);
fprintf(fileID, '_extrude\n_last\n\n%d\n', -1*H);

% Bearing hole; Draw a circle at 0,0,H/6, with radius Rbearing, and extrude it -Z by Hbearing
fprintf(fileID, '_circle 0,0,%f %f\n', H / 6, Rbearing);
fprintf(fileID, '_extrude\n_last\n\n%d\n', Hbearing);
fprintf(fileID, '_circle 0,0,%f %f\n', -H / 6, Rbearing);
fprintf(fileID, '_extrude\n_last\n\n%d\n', -Hbearing);

% Potentiometer slot; Draw profile at 0,0,H/2, and extrude it -Z by H
fprintf(fileID, '_3dpoly\n');
for i = 1:length(PMX)
    fprintf(fileID, '%f,%f,%f\n', PMX(i), PMY(i), H / 6+Hbearing+1);
end
fprintf(fileID, 'c\n'); % Close the polyline
fprintf(fileID, '_extrude\n_last\n\n%d\n', H/3);

% Tooth
for i = 1:size(toothall, 2) % number of tooth
    x = toothall(1, i);
    y = toothall(2, i);
    z = toothall(3, i);
    % Spherical tooth creation
    fprintf(fileID, '_sphere 0,0 %f\n', Rtooth);  % Create sphere at origin
    fprintf(fileID, '_move\n');  % Initiate move command
    fprintf(fileID, '_last\n');  % Select the last created object
    fprintf(fileID, '\n');  % Confirm selection
    fprintf(fileID, '0,0,0\n');  % Specify base point for move
    fprintf(fileID, '%f,%f,%f\n', x, y, z);  % Specify second point (destination)
end

% Mounting hole for arm; Draw a circle in the YZ plane at -L,0,0, radius Rarm and extrude it +Y by L
fprintf(fileID, 'ucs y\n\n'); % Align UCS to make drawing in YZ plane easier
fprintf(fileID, '_circle 0,0,0 %f\n', Rarm);
fprintf(fileID, '_extrude\n_last\n\n%f\n', Larm);
fprintf(fileID, 'ucs w\n'); % Reset UCS
fprintf(fileID, '_move\n');  % Initiate move command
fprintf(fileID, '_last\n');  % Select the last created object
fprintf(fileID, '\n');  % Confirm selection
fprintf(fileID, '0,0,0\n');  % Specify base point for move
fprintf(fileID, '%f,0,0\n', -L);  % Specify second point (destination)

% Mounting outer perimeter; Draw a circle in the YZ plane at -L,0,0, radius R
fprintf(fileID, 'ucs y\n\n'); % Align UCS to make drawing in YZ plane easier
fprintf(fileID, '_circle 0,0,0 %f\n', R);
fprintf(fileID, 'ucs w\n'); % Reset UCS
fprintf(fileID, '_move\n');  % Initiate move command
fprintf(fileID, '_last\n');  % Select the last created object
fprintf(fileID, '\n');  % Confirm selection
fprintf(fileID, '0,0,0\n');  % Specify base point for move
fprintf(fileID, '%f,0,0\n', -L);  % Specify second point (destination)

% Head rectangle; Draw polyline for [zeros(size(heady)); heady; headz]
fprintf(fileID, '_3dpoly\n');
for i = 1:length(heady)
    fprintf(fileID, '%f,%f,%f\n', headx(i), heady(i), headz(i));
    if i < length(heady)
        fprintf(fileID, '%f,%f,%f\n', headx(i+1), heady(i+1), headz(i+1));
    end
end
fprintf(fileID, 'c\n'); % Close the polyline

% Head slot
fprintf(fileID, '_3dpoly\n');
for i = 1:length(heady)
    fprintf(fileID, '%f,%f,%f\n', headx(i), heady(i), headz(i)/3);
    if i < length(heady)
        fprintf(fileID, '%f,%f,%f\n', headx(i+1), heady(i+1), headz(i+1)/3);
    end
end
fprintf(fileID, 'c\n'); % Close the polyline
fprintf(fileID, '_extrude\n_last\n\n%d\n', -1*mean(headx)-L+Larm-0.1);

% % Cosmetic cuts;
% fprintf(fileID, 'ucs y\n\n'); % Align UCS to make drawing in YZ plane easier
% fprintf(fileID, '_circle 0,0,0 %f\n', H/6);
% fprintf(fileID, '_extrude\n_last\n\n40\n');
% fprintf(fileID, 'ucs w\n'); % Reset UCS
% fprintf(fileID, '_move\n');  % Initiate move command
% fprintf(fileID, '_last\n');  % Select the last created object
% fprintf(fileID, '\n');  % Confirm selection
% fprintf(fileID, '0,0,0\n');  % Specify base point for move
% fprintf(fileID, '%f,%f,0\n', -L, max(XY_mm(2,:)));  % Specify second point (destination)
% fprintf(fileID, '_copy\n');  % Initiate move command
% fprintf(fileID, '_last\n');  % Select the last created object
% fprintf(fileID, '\n');  % Confirm selection
% fprintf(fileID, '0,0,0\n');  % Specify base point for move
% fprintf(fileID, '0,%f,0\n', 2*min(XY_mm(2,:)));  % Specify second point (destination)


% Set facet resolution of .stl file to the maximum value (10)
fprintf(fileID, 'facetres\n'); 
fprintf(fileID, '10\n'); 

% Close the file
fclose(fileID);
