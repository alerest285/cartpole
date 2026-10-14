function p = cp_derive(p)
% Recompute derived constants after editing measured physical parameters.
p.a = p.K_g*p.K_t/(p.R_m*p.r_mp);
p.b = p.K_g^2*p.K_t*p.K_m/(p.R_m*p.r_mp^2);
p.m_rot = double(p.rotor_mass)*p.J_m*p.K_g^2/p.r_mp^2;
p.M = p.m_c+p.m_rot+p.m_p;
p.J = (1+p.I_factor)*p.m_p*p.l^2;
p.ml = p.m_p*p.l;
end
