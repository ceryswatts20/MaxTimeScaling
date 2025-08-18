function c = CVector(m, L, q, qdot)
% Create the c vector containing the Coriolis and centripetal torques
% of a 2 Link robotic manipulator

% Input arguments:
%   m = vector of link masses i.e [link 1 mass, link 2 mass, ...]
%   L = vector of link lengths i.e [link 1 length, link 2 length, ...]
%   q = vector of joint positions
%   qdot = vector of joint velocities

% sin = radian, sind = degrees
c1 = -m(2)*L(1)*L(2)*sin(q(2))*(2*qdot(1)*qdot(2) + qdot(2)^2);
c2 = m(2)*L(1)*L(2)* qdot(1)^2*sin(q(2));

c = [c1; c2];

end