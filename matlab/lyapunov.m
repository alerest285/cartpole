% Body for the MATLAB Function block in swingup_lyapunov.slx (handout step 5).
% Paste between the function line and its end. The template's hint: output
% |u| = 1 in the active regions and set the amplitude with the Tunable Gain block
% (start around 4-6 V; the Python energy controller reaches upright in ~2.5 s).
%
% Why: E = m g r (cos a + 1) + 1/2 (J_p + m r^2) a'^2 and the pole equation
% (J_p + m r^2) a'' - m g r sin a = m r x'' cos a give dE/dt = m r cos(a) a' x''.
% So x'' with the sign of a' cos(a) can only add energy. Stop once the energy
% reaches the upright value 2 m g r, and coast above horizontal (handout hint)
% so the pole is not overshot.
function u = lyapunov(alpha_dot, alpha)
    m = 0.127; r = 0.1778; g = 9.81; J = 4/3*m*r^2;      % J = J_p + m r^2 for a uniform rod
    E  = m*g*r*(cos(alpha) + 1) + 0.5*J*alpha_dot^2;
    Eup = 2*m*g*r;
    if abs(alpha) < pi/2 || E >= Eup
        u = 0;
    elseif abs(alpha_dot) < 0.05 && abs(alpha) > 3.1
        u = 1;                                   % hanging still: one push to start
    else
        u = sign(alpha_dot*cos(alpha));
    end
end
