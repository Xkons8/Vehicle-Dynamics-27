function ev = fsae_events(p, which)
% FSAE_EVENTS  Simulate the dynamic events for car p, ready for fsae_points.
%   ev = fsae_events(p)                          all four events
%   ev = fsae_events(p, {'accel', 'skidpad'})    only these (faster)
%   [pts, brk] = fsae_points(fsae_events(p), fsae_field(p.competition))
%
%   accel      75 m from rest (accel_time)
%   skidpad    one lap of the skidpad circle at the highest speed the car can
%              hold there (corner_speed 'hold': grip left after the rear tires
%              push against drag and tire slip drag)
%   autocross  the digitized autocross course from rest (lap_sim)
%   endurance  the car cannot run flat out for the whole event: the pack
%              would run out. So it runs under a power cap, the highest cap
%              at which the endurance distance uses exactly the energy
%              budget, and the lap time and energy come from one lap at it:
%                budget = pack_usable_f * margin * E_pack       (p.scenario)
%                energy = laps * (drive energy - recovered braking energy)
%                laps   = p.scenario.endurance_m / endurance lap length
%              The cap is found by searching between a low cap and full
%              power (energy rises with the cap), to within 0.01 % of the budget.
%
% OUTPUT ev (what fsae_points reads, then extras for reports)
%   .accel .skidpad .autocross   simulated times [s]
%   .endurance_lap               lap time at the power cap [s]
%   .energy_lap_kWh              energy per lap at the cap [kWh]
%   .time_factor                 p.scenario.time_factor_sprint / _endurance:
%                                the simulation drives a perfect lap, so
%                                fsae_points scores these times multiplied by it
%   .cap_W                       endurance power cap [W] (NaN if the pack cannot
%                                cover the distance even at the lowest cap)
%   .budget_kWh, .laps_energy, .energy_event_kWh   the energy balance above

if nargin < 2 || isempty(which)
    which = {'accel', 'skidpad', 'autocross', 'endurance'};
end
R    = fsae_rules();
here = vd_root();
ev = struct('time_factor', struct('sprint', p.scenario.time_factor_sprint, ...
                                  'endurance', p.scenario.time_factor_endurance));

if any(strcmp(which, 'accel'))
    ev.accel = accel_time(p, R.accel_m);
end
if any(strcmp(which, 'skidpad'))
    ev.skidpad = 2*pi*R.skidpad_R_m / corner_speed(p, 1/R.skidpad_R_m, [], 'hold');
end
if any(strcmp(which, 'autocross'))
    [s, k] = load_track(fullfile(here, 'tracks', 'track_autocross.csv'));
    [~, ev.autocross] = lap_sim(p, s, k, 0, false);
end
if any(strcmp(which, 'endurance'))
    [s, k] = load_track(fullfile(here, 'tracks', 'track_endurance.csv'));
    ev = endurance(ev, p, s, k);
end
end


function ev = endurance(ev, p, s, k)
CAP_LOW = 8e3;          % lowest power cap searched [W]
TOL     = 1e-4;         % energy tolerance, fraction of the budget [-]

sc     = p.scenario;
regen  = sc.regen_capture * sc.regen_rt;
laps   = sc.endurance_m / s(end);
budget = sc.pack_usable_f * sc.margin * p.E_pack_Wh / 1000;      % [kWh]
lap    = @(cap) one_lap(p, s, k, cap, regen);                     % [t, kWh per lap]

ev.budget_kWh  = budget;
ev.laps_energy = laps;

% Full power fits the budget: no cap needed.
c_hi = p.P_max;  [t_hi, e_hi] = lap(c_hi);  g_hi = laps*e_hi - budget;
if g_hi <= 0
    ev = store(ev, c_hi, t_hi, e_hi, laps);
    return
end
c_lo = CAP_LOW;  [t_lo, e_lo] = lap(c_lo);  g_lo = laps*e_lo - budget;
if g_lo > 0
    vd_warn('fsae_events:noEnduranceCap', ['The pack cannot cover the endurance ' ...
            'distance even at a %.0f kW power cap, so endurance is not scored.'], CAP_LOW/1e3);
    ev = store(ev, NaN, NaN, NaN, laps);
    return
end

% Regula falsi (Illinois variant) on energy(cap) = budget.
side = 0;
for it = 1:40
    c = c_hi - g_hi * (c_hi - c_lo) / (g_hi - g_lo);
    [t, e] = lap(c);  g = laps*e - budget;
    if abs(g) <= TOL*budget || (c_hi - c_lo) < 1, break; end
    if g > 0
        c_hi = c;  g_hi = g;
        if side == 1, g_lo = g_lo/2; end
        side = 1;
    else
        c_lo = c;  g_lo = g;
        if side == -1, g_hi = g_hi/2; end
        side = -1;
    end
end
ev = store(ev, c, t, e, laps);
end


function [t, e] = one_lap(p, s, k, cap, regen)
% Flying endurance lap at power cap [W]: time [s] and net energy [kWh].
p.P_max = min(p.P_max, cap);
[~, t, E] = lap_sim(p, s, k, [], true);
e = (E.drive_acc_Wh - E.brake_wheel_Wh * regen) / 1000;
end


function ev = store(ev, cap, t, e, laps)
ev.cap_W            = cap;
ev.endurance_lap    = t;
ev.energy_lap_kWh   = e;
ev.energy_event_kWh = e * laps;
end
