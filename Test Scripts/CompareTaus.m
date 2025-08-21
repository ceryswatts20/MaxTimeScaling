%% Time Scaling Algorithm
% sign - Gets sign of each element in an array i.e
%        x = [-3, 0, 5];
%        sign(x)  % → [-1, 0, 1]

% find - Find indices of true elements in an array i.e
%        a = [0, 5, 0, 3];
%        find(a)         % → [2 4]
%        find(a > 2)     % → [2 4]
%        find(a > 2, 1)  % → 2 (first match)

% diff - Computes the difference between adjacent elements of a vector.
%      ie. x = [1, 4, 9];
%          diff(x)  % → [3, 5]

addpath(genpath('C:\Users\40229353\MATLAB Drive\PhD\MaxTimeScaling'))
clear all
close all

% Min joint torques
min_tau1 = [-800; -500];
% Max joint torques
max_tau1 = [500; 700];
% Min joint torques
min_tau2 = [-8000; -5000];
% Max joint torques
max_tau2 = [10000; 14000];
% Initial State - (s, sdot) = (0, 0)
s_i = [0 0];
% Simulation time
tspan = [0 10];

% Plot VLC
% Inital x2 value
sdot_0 = 40;
% Lots of x1 values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau1, max_tau1, s_star, sdot), sdot_0), s_star);
sdot_vlc2 = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau2, max_tau2, s_star, sdot), sdot_0), s_star);
vlc1 = [s_star', sdot_vlc'];
vlc2 = [s_star', sdot_vlc2'];

% Set event conditions
backFunc1 = @(t, x) backwardsStopEvent(t, x, vlc1);
backFunc2 = @(t, x) backwardsStopEvent(t, x, vlc2);
options = odeset('Events', backFunc1, 'RelTol', 1e-6, 'AbsTol', 1e-8);
options2 = odeset('Events', backFunc2, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'backwardMin';

% [Time, Function Output, Points where event occurred, Solutions to tF, ???]
[~, F1] = ode45(@(t, x) Simulation(t, x, direction, min_tau1, max_tau1), tspan, ...
    [1 0], options);
[~, F2] = ode45(@(t, x) Simulation(t, x, direction, min_tau2, max_tau2), tspan, ...
    [1 0], options2);

forwardFunc = @(t, x) forwardsStopEvent(t, x, vlc1, F1);
forwardFunc2 = @(t, x) forwardsStopEvent(t, x, vlc2, F2);
for_options = odeset('Events', forwardFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
for_options2 = odeset('Events', forwardFunc2, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'forwardMax';
[~, A1] = ode45(@(t, x) Simulation(t, x, direction, min_tau1, max_tau1), tspan, ...
    s_i, for_options);
[~, A2] = ode45(@(t, x) Simulation(t, x, direction, min_tau2, max_tau2), tspan, ...
    s_i, for_options2);

% Target value - intersection point
s_target = A1(end, 1);
sdot_interp = interp1(F1(:, 1), F1(:, 2), s_target);
s_target2 = A2(end, 1);
sdot_interp2 = interp1(F2(:, 1), F2(:, 2), s_target2);
% Find rows where s @ F > A(end, 1)
idx = find(F1(:, 1) > s_target);
idx2 = find(F2(:, 1) > s_target2);
% Truncated F curve up to A intersection point
F_trunc = [F1(idx, :); s_target, sdot_interp];
F_trunc2 = [F2(idx2, :); s_target2, sdot_interp2];

% Convert vectors to string form: [a, b]
min_str1 = sprintf('[%d, %d]', min_tau1(1), min_tau1(2));
max_str1 = sprintf('[%d, %d]', max_tau1(1), max_tau1(2));
min_str2 = sprintf('[%d, %d]', min_tau2(1), min_tau2(2));
max_str2 = sprintf('[%d, %d]', max_tau2(1), max_tau2(2));

hold on
% Tau 1 values
plot(vlc1(:, 1), vlc1(:, 2), 'DisplayName', ['VLC \tau_{min} = ' min_str1 ', \tau_{max} = ' max_str1])
plot(F_trunc(:, 1), F_trunc(:, 2), 'DisplayName', ['F \tau_{min} = ' min_str1 '\tau_{max} = ' max_str1])
plot(A1(:, 1), A1(:, 2), 'DisplayName', ['A \tau_{min} = ' min_str1 '\tau_{max} = ' max_str1])
% Tau 2 values
plot(vlc2(:, 1), vlc2(:, 2), 'DisplayName', ['VLC \tau_{min} = ' min_str2 ', \tau_{max} = ' max_str2])
plot(F_trunc2(:, 1), F_trunc2(:, 2), 'DisplayName', ['F \tau_{min} = ' min_str2 '\tau_{max} = ' max_str2])
plot(A2(:, 1), A2(:, 2), 'DisplayName', ['A \tau_{min} = ' min_str2 '\tau_{max} = ' max_str2])
% Intersection points
plot(A1(end, 1), A1(end, 2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 's_1')
plot(A2(end, 1), A2(end, 2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', 's_2')

grid on
legend()
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')
% Set title
title('Comparing different sets of \tau_{min} and \tau_{max}')

% Go up one folder, then into Graphs
graphsFolder = fullfile('..','Graphs');
% Saves figure as MATLAB code to recreate it
saveas(gcf, fullfile('Graphs\CompareTaus.png'))