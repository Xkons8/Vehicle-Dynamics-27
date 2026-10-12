function r = fsae_rules()
% FSAE_RULES  Competition constants: event geometry, power cap, scoring.
%   r = fsae_rules();
%
% Values follow the FSAE Rules 2026 V1.0 (unchanged in 2027 V1.0). Rule numbers
% are given so each value can be checked against the rulebook. These are
% properties of the competition, not of a car - change them only when the
% rulebook changes. The scoring constants are also checked against every
% published 2026 Michigan EV score by vd_selftest.

r.rules_version      = 'FSAE Rules 2026 V1.0';

% --- tractive system ------------------------------------------------------
r.P_max_W            = 80e3;     % EV.3.3.1  max power at the Energy Meter [W]

% --- event geometry -------------------------------------------------------
r.accel_m            = 75;       % D.9.1.1   start line to finish line [m]
r.accel_staging_m    = 0.30;     % D.9.2.3   foremost part of car behind the start line [m]
r.skidpad_R_m        = 9.125;    % D.10.1.1  path centre radius = 15.25/2 + 3.0/2 [m]
r.endurance_m        = 22000;    % D.12.1.3  approximate total distance [m]

% --- timed-event scoring (D.9 accel, D.10 skidpad, D.11 autocross, D.12 endurance)
% points = p_min + p_var * (Tmax/T - 1) / (Tmax/Tmin - 1),   Tmax = t_max_factor * Tmin
% (skidpad squares both ratios). A run slower than Tmax scores p_min.
r.score.accel     = struct('t_max_factor', 1.50, 'p_var',  95.5, 'p_min', 4.5, 'squared', false);
r.score.skidpad   = struct('t_max_factor', 1.25, 'p_var',  71.5, 'p_min', 3.5, 'squared', true);
r.score.autocross = struct('t_max_factor', 1.45, 'p_var', 118.5, 'p_min', 6.5, 'squared', false);
r.score.endurance = struct('t_max_factor', 1.45, 'p_var', 250.0, 'p_min', 0.0, 'squared', false);
                                 % endurance time: total time, finishers only

% Endurance laps points (D.12.13.3): 1 for starting, 1 per lap completed and
% 2 more on completing lap 11 (the driver change), so 25 for all 22 laps.
% Element n+1 is the score for n laps.
n_laps = 0:22;
r.endurance_lap_points = 1 + n_laps + 2*(n_laps >= 11);

% --- efficiency scoring (D.13) --------------------------------------------
r.co2_kg_per_kWh     = 0.65;     % D.13.4.1  EV energy -> CO2
r.co2_ref_kg_per_km  = 0.2002;   % D.13.4.5  EV reference 20.02 kg / 100 km (sets EF_min)
r.ef_min_time_factor = 1.45;     % D.13.4.5  T_your = 1.45 x T_min when computing EF_min
r.efficiency_points  = 100;      % D.13     points for the best efficiency factor
end
