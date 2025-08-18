%% Time Scaling Algorithm
addpath(genpath('C:\Users\40229353\MATLAB Drive\PhD\MaxTimeScaling'))
clear all
close all

% Min joint torques
min_tau = [-5; -8];
% Max joint torques
max_tau = [10; 10];
% Simulation time
tspan = [0 10];

% Plot VLC
% Inital x2 value
sdot_0 = 50;
% Lots of x1 values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, max_tau, s_star, sdot), sdot_0), s_star);

plot(s_star, sdot_vlc, 'DisplayName', 'VLC')
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')
title('Time-scaling Algoritm')
grid on
hold on

p = polyfit(s_star, sdot_vlc , 10);
% Evaluate the polynomial fit at the s_star values
sdot_fit = polyval(p, s_star);
sdot_fit = sdot_fit - 0.04;
% Plot the polynomial fit
%plot(s_star, sdot_fit, 'DisplayName', 'Polynomial Fit');

direction = 'backwardMin';
% [Time, Function Output, Time of events, Solution at events, Index of which event]
[~, F] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, [1 0]);
direction = 'forwardMax';
% [Time, Function Output, Time of events, Solution at events, Index of which event]
[~, A] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, [0 0]);

% plot(A(:, 1), A(:, 2), 'DisplayName', 'A')
% plot(F(:, 1), F(:, 2), 'DisplayName', 'F')

legend show

