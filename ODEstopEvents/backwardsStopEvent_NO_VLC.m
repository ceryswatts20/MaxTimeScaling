% Event Function to Stop Integration
function [value, isterminal, direction] = backwardsStopEvent_NO_VLC(t, x)
    % COMMENT TODO %

    % Stop if s = 0
    value = x(1);
    % Stop integration when the condition is met
    isterminal = 1;
    % Detect any crossing
    direction = 0;
end