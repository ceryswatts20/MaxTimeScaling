function M = MassMatrix(m, L, q)
% Creates the mass matrix for a 2 Link robotic manipulator

% Input arguments:
%   m = vector of link masses i.e [link 1 mass, link 2 mass, ...]
%   L = vector of link lengths i.e [link 1 length, link 2 length, ...]
%   q = vector of joint positions

% cos = radians, cosd = degrees
M11 = m(1)*L(1)^2 + m(2)*(L(1)^2 + 2*L(1)*L(2)*cos(q(2)) + L(2)^2);
M12 = m(2)*(L(1)*L(2)*cos(q(2)) + L(2)^2);
M21 = M12;
M22 = m(2)*L(2)^2;

M = [M11 M12; M21 M22];

end