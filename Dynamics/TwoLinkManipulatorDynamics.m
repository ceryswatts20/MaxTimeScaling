function [m_s, c_s, g_s] = TwoLinkManipulatorDynamics(s, sdot)
% Generates the parameterised lagrangian dynamics
% m - Masses of links
m = [2; 2.5];
% L - Lengths of links
L = [1; 1.5];
% Path start point
q_start = [0; 0];
% Path end point
q_end = [deg2rad(220); deg2rad(200)];

% Path Parameterisation
% q(s) = q_start + s(q_end - q_start)
q_s = q_start + s*(q_end - q_start);
% dq/ds = (q_end - q_start)
q_s_dot = (q_end - q_start);

% New velocity and acceleration vectors:
%   qdot = (q_end - q_start)sdot = dq/ds * sdot
%   qddot = (q_end - q_start)sddot = dq/ds *sddot
qdot = q_s_dot*sdot;

% Generate path parameterised Lagrangian Dynamics
% M(q(s))
M = MassMatrix(m, L, q_s);
% c(q(s), q(s)dot*sdot)
c = CVector(m, L, q_s, qdot);
% g(q(s))
g = GravitationalVector(m, L, q_s);

% tau = m(s)sddot + c(s)sdot^2 + g(s)
% m(s) = M(q(s))*dq/ds
m_s = M*q_s_dot;
% c(s)sdot^2 = c(q(s), q(s)dot*sdot)
c_s = c;
% g(s) = g(q(s))
g_s = g;

end