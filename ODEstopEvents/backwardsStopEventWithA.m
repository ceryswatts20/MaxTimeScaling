% Event Function to Stop Integration
function [value, isterminal, direction] = backwardsStopEventWithA(t, x, vlc, A)
    % COMMENT TODO %
    vlc_values = interp1(vlc(:, 1), vlc(:, 2), x(1), "linear");
    A_values = interp1(A(:, 1), A(:, 2), x(1), "linear");

    % Stop if s = 0 or L = U or F = A
    value = [x(1); x(2) - vlc_values; x(2) - A_values];
    % Stop integration when either condition is met
    isterminal = [1; 1; 1];
    % Detect any crossing
    direction = [0; 0; 0];
end