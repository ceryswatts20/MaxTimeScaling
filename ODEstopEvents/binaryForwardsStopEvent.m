% Event Function to Stop Integration
function [value, isterminal, direction] = binaryForwardsStopEvent(t, x, vlc, F)
    % COMMENT TODO %
    vlc_values = interp1(vlc(:, 1), vlc(:, 2), x(1), "linear");
    F_values = interp1(F(:, 1), F(:, 2), x(1), "linear");

    % Stop if sdot = 0, L = U for F = A
    value = [x(2); x(2) - vlc_values; x(2) - F_values];
    % Stop integration when any conditions are met
    isterminal = [1; 1; 1];
    % Detect any crossing
    direction = [0; 0; 0];
end