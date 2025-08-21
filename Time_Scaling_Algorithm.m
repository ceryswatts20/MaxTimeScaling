%% Time Scaling Algorithm
% Allows access to functions in all folders and subfolders in the path
addpath(genpath('C:\Users\40229353\MATLAB Drive\PhD\MaxTimeScaling'))
clearvars
close all
clear all

% Min joint torques
min_tau = [-800; -500];
% Max joint torques
max_tau = [700; 500];

%% Plot VLC
% Inital sdot value
sdot_0 = 10;
% Lots of s values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, max_tau, s_star, sdot), sdot_0), s_star);

p = polyfit(s_star, sdot_vlc , 10);
% Evaluate the polynomial fit at the s_star values
sdot_fit = polyval(p, s_star);
sdot_fit = sdot_fit - 0.04;
% Save Velocity Limit Curve
%vlc = [s_star', sdot_fit'];
vlc = [s_star', sdot_vlc'];

% Plot the velocity limit curve
plot(vlc(:, 1), vlc(:, 2), 'DisplayName', 'VLC')
xlabel('s')
ylabel('$\dot{s}$', 'Interpreter', 'latex')
title('Time-scaling Algoritm')
grid on
hold on

%% Step 1
% List of switch points
S = [];
% Switch counter
i = 0;
% Initial State - (s, sdot) = (0, 0)
s_i = [0 0];
% Simulation time
tspan = [0 10];

%% Step 2 - Integrate sddot = L(s, sdot) from (1, 0)
% Set event conditions
backFunc = @(t, x) backwardsStopEvent(t, x, vlc);
back_options = odeset('Events', backFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'backwardMin';
% [Time, Function Output, Time of events, Solution at events, Index of which event]
[~, F] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, ...
    [1 0], back_options);

while true
    %% Step 3 - Integrate sddot = U(s, sdot) from s_i
    % Stop if L(s, sdot) = U(s, sdot) or s = 1
    forwardFunc = @(t, x) forwardsStopEvent(t, x, vlc, F);
    for_options = odeset('Events', forwardFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
    direction = 'forwardMax';
    % [Time, Function Output, Time of events, Solution at events, Index of which event]
    [~, A, ~, yA, iA] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, ...
        s_i, for_options);

    % If A crosses F, increment i
    if iA == 3
        % Increment i
        i = i + 1;
        % Set s_i to the crossing point
        s_i = A(end, :);
        % Append s_i to list of switches S
        S = [S; s_i(1)];

        % Target value - intersection point
        s_target = s_i(1);
        % Get sdot value of F at intersection point
        sdot_interp = interp1(F(:, 1), F(:, 2), s_target);
        % Find rows where s @ F > A(end, 1)
        idx = find(F(:, 1) > s_target);
        % Truncated F curve up to A intersection point
        F_trunc = [F(idx, :); s_target, sdot_interp];

        % Plot the functions and crossing
        plot(F_trunc(:, 1), F_trunc(:, 2), 'DisplayName', 'F');
        % A up to intersection
        plot(A(:, 1), A(:, 2), 'DisplayName', sprintf('A_%d', i - 1));
        % F-A crossing point
        plot(s_i(1), s_i(2), 'go', 'MarkerFaceColor', 'g', ...
            'DisplayName', sprintf('s_%d', i));

        % Exit the loop
        break;
    else
        % Save point where VLC is crossed
        s_lim = A(end, :);
        plot(A(:, 1), A(:, 2))

        %% Step 4 - Binary Search
        % Initialize binary search parameters
        tolerance = 1e-5;
        % sdot_high = sdot_lim
        sdot_high = s_lim(2);
        sdot_low = 0;

        % Binary search loop
        while (sdot_high - sdot_low) > tolerance
            % Set the initial guess for the binary search
            sdot_test = (sdot_high + sdot_low) / 2;
            % Test point
            test_point = [s_lim(1), sdot_test];

            % Integrate s_ddot = L(s, sdot) forward in time from (s_lim, sdot_test)
            direction = 'forwardMin';
            % Set event conditions - Stop if crosses VLC, F or s = 1
            binaryFunc = @(t, x) binaryForwardsStopEvent(t, x, vlc, F);
            binaryForwardOptions = odeset('Events', binaryFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
            [~, A_test, ~, ~, iA] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), ...
                tspan, test_point, binaryForwardOptions);

            % Check if the curve crosses vlc
            if iA == 2
                % Adjust upper bounds
                sdot_high = sdot_test;
            % Check if the curve hits sdot = 0
            elseif iA == 1
                % Adjust lower bounds
                sdot_low = sdot_test;
            % If curve crosses F
            elseif iA == 3
                error('crossed F during binary search')
            else
                error('sddot = L(s, sdot) forward in time does not cross vlc or sdot=0');
            end
        end

        % Final result
        s_tan = test_point;

        %% Step 5
        back = @(t, x) backwardsDeccelStopEvent(t, x, A);
        b_options = odeset('Events', back, 'RelTol', 1e-6, 'AbsTol', 1e-8);
        direction = 'backwardMin';
        [~, Ai, ~, yAi, iAi] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), ...
                tspan, s_tan, b_options);

        % Increment i
        i = i + 1;
        % Save intersection point
        s_i = [Ai(end, 1), Ai(end, 2)];
        % Plot latest switch point
        plot(s_i(1), s_i(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', sprintf('s_%d', i));

        plot(Ai(:, 1), Ai(:, 2),'b', 'DisplayName', sprintf('A_%d', i))
        valid_idx = (A(:, 1) <= s_i(1)) & (A(:, 2) <= s_i(2));
        % Filter A curve to up to intersection point
        filtered_A = A(valid_idx, :);
        % Add intersection point filtered curve A
        filtered_A = [filtered_A; s_i];
        % Plot the filtered curve A
        plot(filtered_A(:, 1), filtered_A(:, 2), 'DisplayName', sprintf('A_%d', i - 1))
        % Append intersection point to list of switches
        S = [S; s_i(1)];

        %% Step 6
        % Increment i
        i = i + 1;
        % Save tangent point (result of binary search)
        s_i = s_tan
        % Append point to list of switches
        S = [S; s_i(1)];
        % Plot latest switch point
        plot(s_i(1), s_i(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', sprintf('s_%d', i));
    end
end

legend()


