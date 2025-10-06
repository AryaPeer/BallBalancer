%% Animate the motion of the mechanism 

% Initial Clears
clear; 
clc; 
close all;

%% Initial Parameters
l = 76;
l_12 = 25;
l_11 = 65;

h = 19;
w = 56;

theta_11 = 0:1:720*10; % Two rotations from 0 to 360 with step 1: [0,1,2,3,4....360]

% Setting up figure
figure(1);
hold on;
grid on;
title("Animated Mechanism");
xlabel('i Position (m)');
ylabel('j Position (m)');
axis([-200, 200, 0, 200]); % Set axis bounds
ylim([-0.7, 0.7]); % Fix y-axis limits
axis equal; % Maintain equal scaling for x and y axes
legend on;
legend('Location', 'southeast'); % Predefined legend location

% Setting up fixed pivot A and B:
plot(-w, h, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Pivot A');
plot(0, 0, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 10, 'DisplayName', 'Pivot B');

% Setting up links
linkDC_start = [0, 0]; % Link DC on one end is always connected to the origin
linkDC_end = [NaN, NaN];
linkCB_start = [NaN, NaN]; 
linkCB_end = [NaN, NaN];
linkBD_start = [NaN, NaN];
linkBD_end = [NaN, NaN]; 

% Initializing link plots
linkDC_plot = plot(linkDC_start(1), linkDC_end(2), 'b', 'LineWidth', 2, 'DisplayName', 'Link AB');
linkCB_plot = plot(linkDC_end(1), linkCB_end(2), 'b', 'LineWidth', 2, 'DisplayName', 'Link AC');
linkBD_plot = plot(linkCB_end(1), linkBD_end(2), 'b', 'LineWidth', 2, 'DisplayName', 'Link BD');

% Setting up sliders
%sliderC_pos = [NaN, NaN];
%sliderD_pos = [NaN, r1]; % Slider D always remains grounded on the same j level

% Initializing slider plots
%sliderD_plot = plot(sliderD_pos(1), sliderD_pos(2), 'ro', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'none', 'MarkerSize', 10, 'DisplayName', 'Slider D');
%sliderC_plot = plot(sliderC_pos(1), sliderC_pos(2), 'ro', 'MarkerEdgeColor', 'r', 'MarkerFaceColor', 'none', 'MarkerSize', 10, 'DisplayName', 'Slider C');

% Updating animation for two full rotations
for i = 1:length(theta_11)

    % Getting the current angle
    theta_11 = i;

    % Calculating new linear and angular displacement values
    theta = 
    theta_12 = arccosd([l_11*cosd(theta_11) + w - l * cosd(theta)] / l_12);
    
    % Updating link positions based on calculated values
    linkAB_end = [cosd(theta2)*r2, sind(theta2)*r2]; % Link AB has constant length, therefore its end points depend on theta2
    linkAC_end = [cosd(theta6)*r6, sind(theta6)*r6]; % Link AC has constant length, therefore its end points depend on theta6
    linkBD_start = [cosd(theta2)*r2, sind(theta2)*r2]; % Link AC starts at Link AB's end
    linkBD_end(1) = r0; % Link BD ends at the same j everytime but the i depends on the r0

    %Updating slider positions based on calculated values
    %sliderC_pos = [cosd(theta6)*r6, sind(theta6)*r6]; % Slider C is where Link AC ends
    %sliderD_pos(1) = r0; % Slider D has the same j but moves across the i with r0

    set(gca, 'YLim', [-0.7, 0.7]); 

    % Update plots for links
    set(linkAB_plot, 'XData', [linkAB_start(1), linkAB_end(1)], 'YData', [linkAB_start(2), linkAB_end(2)]);
    set(linkAC_plot, 'XData', [linkAC_start(1), linkAC_end(1)], 'YData', [linkAC_start(2), linkAC_end(2)]);
    set(linkBD_plot, 'XData', [linkBD_start(1), linkBD_end(1)], 'YData', [linkBD_start(2), linkBD_end(2)]);

    % Update plots for sliders
 %   set(sliderC_plot, 'XData', sliderC_pos(1), 'YData', sliderC_pos(2));
 %   set(sliderD_plot, 'XData', sliderD_pos(1), 'YData', sliderD_pos(2));
    
    % Pause to slow animation
    pause(0.01);
   
end 
