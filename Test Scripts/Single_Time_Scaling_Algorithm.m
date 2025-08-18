%% Time Scaling Algorithm
addpath(genpath('C:\Users\40229353\MATLAB Drive\PhD\MaxTimeScaling'))
clearvars
close all

% Min joint torques
min_tau = [-5; -8];
% Max joint torques
max_tau = [10; 10];

%% Plot VLC
% Inital sdot value
sdot_0 = 0.1;
% Lots of s values
s_star = 0:0.001:1;

% Find sdot for each s
% arrayfun() applies fsolve() to each s value iteratively.
sdot_vlc = arrayfun(@(s_star) fsolve(@(sdot) vlcSimulation(min_tau, ...
    max_tau, s_star, sdot), sdot_0), s_star);

p = polyfit(s_star, sdot_vlc , 15);
% Evaluate the polynomial fit at the s_star values
sdot_fit = polyval(p, s_star);
sdot_fit = sdot_fit - 0.02;
% Save Velocity Limit Curve
vlc = [s_star; sdot_fit]';
% fixed_column = 1 * ones(1001, 1);
% vlc = [s_star', fixed_column];

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
[~, F, tF, gyF, iF] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, [1 0], back_options);

%% Step 3 - Integrate sddot = U(s, sdot) from s_i
% Stop if L(s, sdot) = U(s, sdot) or s = 1
forwardFunc = @(t, x) forwardsStopEvent(t, x, vlc);
for_options = odeset('Events', forwardFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
direction = 'forwardMax';
% [Time, Function Output, Time of events, Solution at events, Index of which event]
[~, A, tA, yA, iA] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), tspan, s_i, for_options);

%% Compare y-values at same x positions
% % A_F_interp = A y-values @ Fs x values
% A_F_interp = interp1(A(:, 1), A(:, 2), F(:, 1), 'linear');
% % If sdot @ F < sdot @ A for the same s then A crossed F
% A_F_diff = F(:, 2) - A_F_interp;
% % Find zero crossings (where sign changes) + x + = +, + x - = -, - x - = +
% % Multiply adjacent elements and check if result is < 0
% % find() -> returns index of A_F_diff that fulfills the condition
% A_F_crossing_indx = find(A_F_diff(1:end-1) .* A_F_diff(2:end) < 0);

% If A crosses F, increment i
% if size(A_F_crossing_indx) ~= 0
if iA == 1
    i = i + 1;
    % % x before crossing
    % before_Crossing_x = F(A_F_crossing_indx,1);
    % % x after crossing
    % after_Crossing_x = F(A_F_crossing_indx+1,1);
    % % y-difference before crossing (F - A)
    % % calculates differences between adjacent elements
    % before_Crossing_y = A_F_diff(A_F_crossing_indx);
    % % y-difference after crossing
    % after_Crossing_y = A_F_diff(A_F_crossing_indx+1);
    % 
    % % crossing_point — the x-value where diff == 0, i.e. where the curves cross.
    % crossing_point_x = before_Crossing_x - before_Crossing_y * (after_Crossing_x - before_Crossing_x) / (after_Crossing_y - before_Crossing_y);
    % % y-value from F at x_cross
    % crossing_point_y = interp1(F(:,1), F(:,2), crossing_point_x);
    % % Set s_i to the crossing point
    % s_i = [crossing_point_x crossing_point_y];
    s_i = [F(end, 1), F(end, 2)];
    % Append s_i to list of switches S
    S = [S; s_i];

    % % Plot the functions and crossing
    % figure;
    % % Truncate F and A up to crossing point
    % F_trunc = F(1:A_F_crossing_indx, :);
    % A_trunc = A(A(:,1) <= crossing_point_x, :);
    % % Add crossing point to the end
    % F_trunc(end+1,:) = [crossing_point_x, crossing_point_y];
    % A_trunc(end+1,:) = [crossing_point_x, crossing_point_y];
    % 
    % % F up to intersection
    % plot(F_trunc(:,1), F_trunc(:,2), 'DisplayName', 'F');
    % % A up to intersection
    % plot(A_trunc(:,1), A_trunc(:,2), 'DisplayName', 'A');
    % % F-A crossing point
    % plot(crossing_point_x, crossing_point_y, 'ro', 'MarkerFaceColor', 'r');

    plot(F(:, 1), F(:, 2), 'DisplayName', 'F')
    plot(A(:, 1), A(:, 2), 'DisplayName', sprintf('A_%d', i))
% Breached the VLC
else
    % Save point where VLC is crossed
    s_lim = A(end, :);

    % Plot F
    plot(F(:,1), F(:,2), 'DisplayName', 'F');
    plot(F(end, 1), F(end, 2), 'ko', 'MarkerFaceColor', 'k', 'DisplayName', 'F-VLC')
    plot(A(:, 1), A(:, 2), 'DisplayName', 'A')

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
        % Set event conditions
        binaryFunc = @(t, x) binaryForwardsStopEvent(t, x, vlc);
        binaryForwardOptions = odeset('Events', binaryFunc, 'RelTol', 1e-6, 'AbsTol', 1e-8);
        [~, A_test, ~, ~, iA_test] = ode45(@(t, x) Simulation(t, x, direction, ...
            min_tau, max_tau), tspan, test_point, binaryForwardOptions);
        %plot(A_test(:, 1), A_test(:, 2))

        % Check if the curve crosses vlc
        if iA_test == 2
            % Adjust upper bounds
            sdot_high = sdot_test;
        % Check if the curve hits sdot = 0
        elseif iA_test == 1
            % Adjust lower bounds
            sdot_low = sdot_test;
        else
            disp('binary search else statement')
        end
    end

    % TODO - COMMENT
    s_tan = [s_lim(1), sdot_test];

    %% Step 5
    back = @(t, x) backwardsDeccelStopEvent(t, x, A);
    b_options = odeset('Events', back, 'RelTol', 1e-6, 'AbsTol', 1e-8);
    direction = 'backwardMin';
    [~, Ai] = ode45(@(t, x) Simulation(t, x, direction, min_tau, max_tau), ...
            tspan, s_tan, b_options);

    % Increment i
    i = i + 1;
    % Save intersection point
    s_i = [Ai(end, 1), Ai(end, 2)];
    % Plot latest switch point
    plot(s_i(1), s_i(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', sprintf('s_%d', i));

    % Plot A_i
    plot(Ai(:, 1), Ai(:, 2), 'DisplayName', sprintf('A_%d', i))
    % Only plot A up to intersection point
    % Find indices where x and y are within bounds
    valid_idx = (A(:, 1) <= s_i(1)) & (A(:, 2) <= s_i(2));
    % Filter points that are before the intersection point
    A = A(valid_idx, :);
    % Add the intersection point to the end of A
    A = [A; s_i];
    plot(A(:, 1), A(:, 2), 'DisplayName', sprintf('A_%d', i - 1))

    % Append s_i to S
    S = [S; s_i(1)];

    %% Step 6
    i = i + 1;
    s_i = s_tan;
    % Store the intersection point in the list of switches S
    S = [S; s_i(1)];
    % Plot switch point
    plot(s_i(1), s_i(2), 'go', 'MarkerFaceColor', 'g', 'DisplayName', sprintf('s_%d', i));

    % Go to step 3
end

legend()


