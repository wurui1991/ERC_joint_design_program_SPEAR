% =========================================================================
% ERC design step 2: cam design
%
% Author: Rui Wu (rui.wu@usys.ethz.ch)
%         Stefano Mintchev (stefano.mintchev@usys.ethz.ch)
% Environmental Robotics Lab, ETH Zurich, 2024
%
% Funded by the Horizon Europe project in AI & robotics:
% "SPEAR: Spatial Perception & Embodied Autonomy Research"
% =========================================================================

%% !! NOTE: run SpringElector.m first before you run this program

%% INPUT: Spring properties

n_spring = 1; % No. of springs
Lmax = 43.5 /1000; % Spring max length (m)
D_L_max = 20 /1000; % Spring max elongation (Tmax/k) (m)
Tmax = 35.09 * n_spring; % Spring max tension * no. of springs (N)

%% INPUT: Resolution (default: 0.1*pi/180)

dtheta = 0.1*pi/180;

%% Derive spring parameters
k=Tmax/D_L_max; % stiffness (N/m) * no. of springs
Lmin=Lmax-D_L_max; % Spring's theoretical length at zero tension (m)
Uspring=0.5*k*D_L_max^2; % spring energy capacity (J)

%% Design initialisation

% Interpolate the target torque according to resolution
theta = (Theta_EA(1):dtheta:Theta_EA(end)); % acquisition points
M = interp1(Theta_EA, M_EA, theta, 'linear'); % target torque (N*m)
DU = cumtrapz(theta*2, M);% incremental energy starting from theta = 0 (J)

% Display SF achieved by the selected spring
SF_achieved = min((Tmax*Lmax)/4/M_slp,Uspring/(max(DU)-min(DU)));
fprintf('\nSF achieved by the selected spring: %.3g\n\n', SF_achieved);
if SF_achieved < 1
    error('SF < 1, design can not proceed!');
end

U0 = Uspring-max(DU); % initial energy at theta(1) (J)
L0 = sqrt(U0*2/k)+Lmin; % initial spring length at theta(1) (m)
U = DU+U0; % spring energy U reset from the initial length Lmin (J)
R = L0/2; % radius of first point of cam (m)
XY = [0;R]; % first point of rolamite in cartesian coordinate
M_actual = nan;
rot = [cos(dtheta) sin(dtheta); -sin(dtheta) cos(dtheta)]; % rot matrix for c.w. dtheta

%% Design

for i=1:length(M)
    if i ~= 1
        XY = rot*XY; % rotate the cam geometry by dtheta, c.w.(clock wise)
    end
    RU = (Lmin+sqrt(2*U(i)/k))/2; % R required to achieve Ui (not necessarily achievable)
    Ri = max(RU,max(XY(2,:))); % apply geometrical constraint to prevent interference from the cam geometry generated in previous steps
    Ti = k*(Ri*2-Lmin); % current spring tension 
    Xi1 = -1*M(i)/Ti; % spring's moment arm required to achieve Mi with length of Ri (not necessarily achievable)
    Xi2 = (R(end)/cos(dtheta)-Ri)/tan(dtheta); % max. moment arm required by geometry to prevent interferencing the previous geometry
    if i ~= 1
        Xi = min(Xi1,Xi2); % Apply geometrical constraint
    else
        Xi = Xi1; % spring's moment arm at the initial step
    end
    R(i) = Ri;
    XY(:,i) = [Xi;Ri];
    M_actual(i) = -1*Ti*Xi; % actual torque achieved by the design
end

% the actual energy curve achieved by the designed ERC
U_actual = 0.5*(R*2-Lmin).^2*k; % energy
% rotate the cam profile so the initial contact is on X axis
remain = pi/2-(Theta_EA(end)-Theta_EA(1))/2; % angle to rotate
rot = [cos(remain) sin(remain); -sin(remain) cos(remain)];
XY = rot*XY;


%% Visualisation

figure % cam profile
plot(XY(1,:)*1000,XY(2,:)*1000,'k','LineWidth', 1.5)
hold on
scatter(0,0,'filled','k') % spring pivot point
fontsize(16,"points")
daspect([1 1 1])
title('ERC profile [mm]')
grid on

figure % torque response
plot(theta*2*180/pi,U,'LineWidth', 1.5)
hold on
plot(theta*2*180/pi,U_actual,'--','LineWidth', 1.5)
xlabel('Rotation angle'); ylabel('Strain energy [J]')
fontsize(16,"points")
legend('Target','Achieved by design')
title('ERC energy response')
grid on

figure % energy response
plot(Theta_EA*180/pi*2,M_EA,'LineWidth', 1.5)
hold on
plot(theta*180/pi*2,M_actual,'--','LineWidth', 2.5)
xlabel('Rotation angle [degree]'); ylabel('Torque [Nm]')
fontsize(16,"points")
legend('Target','Achieved by design')
title('ERC torque response')
grid on