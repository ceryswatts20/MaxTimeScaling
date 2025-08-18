% Event Function to Stop Integration
function [value, isterminal, direction] = forwardsStopEvent_NO_VLC(t, x, F)
    % COMMENT TODO %
    F_values = interp1(F(:, 1), F(:, 2), x(1), "linear");

    % Stop if s = 1 or F = A
    value = [x(1) - 1; x(2) - F_values];
    % Stop integration when any conditions are met
    isterminal = [1; 1];
    % Detect any crossing
    direction = [0; 0];  
end