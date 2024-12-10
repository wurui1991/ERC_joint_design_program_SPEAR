% =========================================================================
% ERC sensor real time visualisation
%
% Author: Rui Wu (rui.wu@usys.ethz.ch)
%         Stefano Mintchev (stefano.mintchev@usys.ethz.ch)
% Environmental Robotics Lab, ETH Zurich, 2024
%
% Funded by the Horizon Europe project in AI & robotics:
% "SPEAR: Spatial Perception & Embodied Autonomy Research"
% =========================================================================
%% !! NOTE: use this program with Arduino program "ERC_sensorised.ino"
%%% You need to change the serial port address based on Arduino IDE %%%
%%% Run this program with Arduino IDE closed %%%

clear

log = 0; % 1: log, 0: no log

%%% Initialize serial communication %%%
%%% You need to change the serial port address based on Arduino IDE %%%
s = serialport('/dev/cu.usbserial-144210', 115200);
configureTerminator(s, "CR/LF");

% Initialize data arrays
F = [];
timeBuffer = [];

% File to save data
if log == 1
    filename = 'instron.csv';
    if isfile(filename)
        delete(filename); % Delete the existing file if it exists
    end
    % Create file and write header
    fid = fopen(filename, 'w');
    fprintf(fid, 'Time,Angle\n'); % Header for time and angle
    fclose(fid);
end

% Initialize torque-angle curve
M_max = 0.05; % Target maximum moment (N*m)
Theta_EA = (-15:0.25:15); % Rotation in degrees
M_EA = sqrt(15^2 - Theta_EA.^2) / 30 + 0.5;
Theta_EA = [0.1 10 Theta_EA + 30]; % Adjusted rotation in degrees
M_EA = [1 1 M_EA] * M_max;
M_EA(end)=M_max;
M_EA = [fliplr(-M_EA) M_EA] * 100; % Convert to N*cm
Theta_EA = [fliplr(-Theta_EA) Theta_EA] * 2; % Adjust angle scaling

t0 = clock;
tp = t0;

% Create figure and subplots
figure_handle = figure('Name', 'Real-Time Angle Monitoring', 'NumberTitle', 'off', 'Position', [100, 100, 1500, 900]);
% Set default font size for the figure
set(figure_handle, 'DefaultAxesFontSize', 15);
set(figure_handle, 'DefaultTextFontSize', 15);

% Adjust positions to make space for the acknowledgment text
subplot_left = subplot(1, 2, 1);
pos_left = get(subplot_left, 'Position');
pos_left(2) = pos_left(2) + 0.05; % Move down
pos_left(4) = pos_left(4) - 0.3; % Reduce height
set(subplot_left, 'Position', pos_left);

subplot_right = subplot(1, 2, 2);
pos_right = get(subplot_right, 'Position');
pos_right(2) = pos_right(2) + 0.05; % Move down
pos_right(4) = pos_right(4) - 0.3; % Reduce height
set(subplot_right, 'Position', pos_right);

% Left subplot: Angle vs. Time
hold(subplot_left, 'on');
grid(subplot_left, 'on'); % Add grid
angle_plot = plot(subplot_left, NaN, NaN, 'b-', 'LineWidth', 2);
xlabel(subplot_left, 'Time [s]');
ylabel(subplot_left, 'Angle [degrees]');
title(subplot_left, 'Real-time joint angle');
angle_text = text(subplot_left, 0, 0, '', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', 15);
ylim([-90 90])

% Right subplot: Torque-Angle Plot
hold(subplot_right, 'on');
grid(subplot_right, 'on'); % Add grid
plot(subplot_right, Theta_EA, M_EA, 'k-', 'LineWidth', 2); % Torque-angle curve
current_angle_line = xline(subplot_right, NaN, 'r--', 'LineWidth', 2);
xlabel(subplot_right, 'Angle [degrees]');
ylabel(subplot_right, 'Torque [N·cm]');
title(subplot_right, 'Stiffness Response');
xlim(subplot_right, [-100 100]);
ylim(subplot_right, [min(M_EA), max(M_EA)]);

% Add acknowledgments above the subplots
annotation('textbox', [0, 0.85, 1, 0.08], ...
    'String', {'Elastic Rolling Cam (ERC)'}, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 25, 'FontWeight', 'bold', 'Interpreter', 'none');
annotation('textbox', [0, 0.81, 1, 0.08], ...
    'String', {'enabling ultra-programmable stiffness'}, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 20, 'FontWeight', 'bold', 'Interpreter', 'none');
annotation('textbox', [0, 0.75, 1, 0.08], ...
    'String', {'Environmental Robotics Lab, ETH Zurich', ...
               'Funded by the Horizon Europe project in AI & Robotics:', ...
               '"SPEAR: Spatial Perception & Embodied Autonomy Research"'}, ...
    'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
    'FontSize', 15, 'FontWeight', 'bold', 'Interpreter', 'none');

% Start the loop
while ishandle(figure_handle)
    % flush(s);
    try
        readline(s); % Purge the first line
        data_line = readline(s); % Read data line
        data = strsplit(strtrim(data_line)); % Split data separated by space
        time = etime(clock, t0); % Time of sampling
        angle_value = str2double(data{1}); % Assuming data{1} is angle value
        if isnan(angle_value)
            error('Invalid angle value');
        end
        F = [F angle_value];
        timeBuffer = [timeBuffer time];
        % Keep data from the last 10 seconds
        idx_keep = timeBuffer >= (time - 10);
        F = F(idx_keep);
        timeBuffer = timeBuffer(idx_keep);

        if log == 1
            % Append new data to the file
            fid = fopen(filename, 'a');
            fprintf(fid, '%f,%f\n', time, angle_value);
            fclose(fid);
        end

    catch
        fprintf('Failed to read or process data from the serial port.\n');
    end

    if etime(clock, tp) >= 0.1 % Faster plotting period for smoother updates
        tp = clock; % Update plotting time
        % Update left subplot
        idx_last_3s = timeBuffer >= (time - 3);
        set(angle_plot, 'XData', timeBuffer(idx_last_3s), 'YData', F(idx_last_3s));
        set(subplot_left, 'XLim', [time - 3, time]);
        if ~isempty(F(idx_last_3s))
            y_min = min(F(idx_last_3s)) - 1;
            y_max = max(F(idx_last_3s)) + 1;
            set(subplot_left, 'YLim', [-100, 100]);
            delete(angle_text);
            angle_text = text(subplot_left, time - 2.9, 90, sprintf('Current Angle: %.1f°', F(end)), 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top', 'FontSize', 15);
        end

        % Update right subplot
        current_angle_deg = F(end);
        set(current_angle_line, 'Value', current_angle_deg);
        drawnow;
    end
end
