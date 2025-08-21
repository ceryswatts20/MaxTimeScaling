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
min_tau = [-800; -500];
% Max joint torques
max_tau = [700; 500];
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
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, max_tau, s_star, sdot), sdot_0), s_star);
vlc = [s_star', sdot_vlc'];
plot(s_star, sdot_vlc, 'DisplayName', 'VLC')
hold on

% Set event conditions
backFunc = @(t, x) backwardsStopEvent_NO_VLC(t, x);
options = odeset('Events', backFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'backwardMin';
% [Time, Function Output, Points where event occurred, Solutions to tF, ???]
[~, F] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, ...
    [1 0], options);

forwardFunc = @(t, x) forwardsStopEvent_NO_VLC(t, x, F);
for_options = odeset('Events', forwardFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'forwardMax';
[~, A, ~, ~, iA] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, ...
    s_i, for_options);

% Target value - intersection point
s_target = A(end, 1);
sdot_interp = interp1(F(:, 1), F(:, 2), s_target);
% Find rows where s @ F > A(end, 1)
idx = find(F(:, 1) > s_target);
% Truncated F curve up to A intersection point
F_trunc = [F(idx, :); s_target, sdot_interp];

hold on
plot(F_trunc(:, 1), F_trunc(:, 2), 'DisplayName', 'F')
plot(A(:, 1), A(:, 2), 'DisplayName', 'A')
grid on
legend()
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')

% Convert vectors to string form: [a, b]
min_str = sprintf('[%d, %d]', min_tau(1), min_tau(2));
max_str = sprintf('[%d, %d]', max_tau(1), max_tau(2));
% Set title
title(['\tau_{min} = ' min_str ', \tau_{max} = ' max_str])

% Go up one folder, then into Graphs
graphsFolder = fullfile('..','Graphs');
% Saves figure as MATLAB code to recreate it
%saveas(gcf, fullfile('Graphs\Looks_correct.png'))