function vlc = vlcSimulationU(min_tau, max_tau, x1, x2)
% Simulate the system to get the Upper acceleration values
u = [0 0];

[m_x1, c_x1, g_x1] = TwoLinkManipulatorDynamics(x1, x2);

for i = 1:2
    if m_x1(i) > 0
        u(i) = (max_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
    end

    if m_x1(i) < 0
        u(i) = (min_tau(i) - c_x1(i) - g_x1(i))/m_x1(i);
    end
end

% U(x1, x2) = min U(x1, x2) -> R.H.S is a vector
U = min(u);

vlc = U;

end

