function vlc = vlcSimulation(min_tau, max_tau, x1, x2)
% Simulate the system to get the Upper and Lower acceleration values
l = [0 0];
u = [0 0];

[m_x1, c_x1, g_x1] = TwoLinkManipulatorDynamics(x1, x2);

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

vlc = L - U;

end

