% Event Function to Stop Integration
function [value, isterminal, direction] = backwardsDeccelStopEvent(t, x, A)
    values = interp1(A(:, 1), A(:, 2), x(1), 'linear', 'extrap');
    %values = interp1(A(:, 1), A(:, 2), x(1), 'linear');

    % Stop if s = 0 or A = values
    value = x(2) - values;
    % Stop integration when either condition is met
    isterminal = 1;
    % Detect any crossing
    direction = 0;  
end