function g = GravitationalVector(m, L, q)
% Creates the gravitational torque vector for a 2 Link robotic manipulator

% Input arguments:
%   m = vector of link masses i.e [link 1 mass, link 2 mass, ...]
%   L = vector of link lengths i.e [link 1 length, link 2 length, ...]
%   q = vector of joint positions

% Gravitational constant
g = 9.81;

% cos = radians, cosd = degrees
g1 = (m(1) + m(2))*L(1)*g*cos(q(1)) + m(2)*g*L(2)*cos(q(1)+q(2));
g2 = m(2)*g*L(2)*cos(q(1)+q(2));

g = [g1; g2];

end