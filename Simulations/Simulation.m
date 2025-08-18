function [xdot] = Simulation(t, x, dir, min_tau, max_tau)
%Simulation Summary of this function goes here
%   Detailed explanation goes here
A = [0 1; 0 0];
B = [0; 1];
l = [0 0];
u = [0 0];

[m_x1, c_x1, g_x1] = TwoLinkManipulatorDynamics(x(1), x(2));
% Acceleration boundaries
% Constraint - min_tau < tau < max_tau
% L(s, sdot) < sddot < U(s, sdot)
% L(x1, x2) < u < U(x1, x2)
for i = 1:2
    if m_x1(i) > 0
        l(i) = (min_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
        u(i) = (max_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
    end

    if m_x1(i) < 0
        l(i) = (max_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
        u(i) = (min_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
    end
end

% L(x1, x2) = max L(x1, x2) -> R.H.S is a vector
L = max(l);
% U(x1, x2) = min U(x1, x2) -> R.H.S is a vector
U = min(u);

% Forwards in time, max acceleration
if strcmp(dir, 'forwardMax')
    xdot = A*x + B*U;
% Backwards in time, max decceleration
elseif strcmp(dir, 'backwardMin')
   xdot = -A*x - B*L;
% Forwards in time, max decceleration
elseif strcmp(dir, 'forwardMin')
    xdot = A*x + B*L;
end

end

