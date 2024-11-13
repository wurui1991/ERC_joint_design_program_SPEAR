% =========================================================================
% ERC Design Step 1: Spring Selection based on target torque and Safety
% Factor (SF)
%
% Author: Rui Wu (rui.wu@usys.ethz.ch)
%         Stefano Mintchev (stefano.mintchev@usys.ethz.ch)
% Environmental Robotics Lab, ETH Zurich, 2024
%
% Funded by the Horizon Europe project in AI & robotics:
% "SPEAR: Spatial Perception & Embodied Autonomy Research"
% =========================================================================

%% Input 1/2: Target Design Alignment Index (DAI)

SF = 2; % default: 2

%% Input 2/2: Required Response

% Rotation of one cam (1/2 of total bending), in radians
Theta_EA = [-45 -44.9 -30 -15 0 0.1 15 30 44.9 45] * pi / 180;  
% Target torque profile (N*m)
M_EA = [-0.4 -0.035 0 -0.07 -0.035 0.035 0.07 0 0.035 0.4];  

%% Spring Requirement Evaluation

% Target incremental energy starting from Theta_EA(1)
DU = cumtrapz(Theta_EA * 2, M_EA);  
% Target torque variation rate
slopes = diff(M_EA) ./ diff(Theta_EA * 2);  
% Maximum target torque reduction rate
M_slp = -1 * min(slopes);  

%% Display
fprintf('Spring requirement:\n');
fprintf('   Max length & tension:       L_max*T_max >= %.3g N*m\n', 4 * M_slp * SF);
fprintf('   Max. elongation & tension: ΔL_max*T_max >= %.3g N*m\n\n', 2 * (max(DU) - min(DU)) * SF);
fprintf('For Max. torque reduction rate = %.3g N*m\n', M_slp);
fprintf('and target energy variation = %.3g J\n', (max(DU) - min(DU)));
fprintf('with Safety Factor SF = %.3g \n', SF);

figure
plot(Theta_EA*2*180/pi,M_EA,'LineWidth', 1.5)
xlabel('Rotation angle [degree]'); ylabel('Torque [Nm]')
fontsize(16,"points")
title('Target ERC torque response')
grid on