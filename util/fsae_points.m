function [total, brk] = fsae_points(ev, F)
% FSAE_POINTS  Dynamic-event points for one car, scored against a real field.
%   [total, brk] = fsae_points(ev, F)
%   [total, brk] = fsae_points(fsae_events(p), fsae_field(p.competition))
%
% INPUTS
%   ev  the car's results. A field that is missing or NaN is not scored and
%       is listed in brk.missing.
%         .accel .skidpad .autocross   best run time [s]
%         .endurance_lap    average endurance lap [s], penalties included
%         .laps             endurance laps completed (default: all, F.laps)
%         .energy_lap_kWh   energy used per endurance lap [kWh]
%         .time_factor      optional: .sprint and .endurance multiply the times
%                           before scoring (fsae_events sets them from
%                           p.scenario; a real, measured time uses 1)
%   F   the competition the car is entered in, from fsae_field
%
% OUTPUTS
%   total  sum of the events that scored
%   brk    points per event: .accel .skidpad .autocross .efficiency, and
%          .endurance = .endurance_time + .endurance_laps;
%          .t_scored (the times after the time factor), .tmin (the fastest
%          time each event was scored against), .ef .ef_min .ef_max .co2_lap,
%          .missing, .complete (true when all five events scored)
%
% HOW THE POINTS WORK (FSAE Rules section D; constants in fsae_rules)
%   Timed events compare your time T with the fastest time Tmin:
%       points = p_min + p_var * (Tmax/T - 1) / (Tmax/Tmin - 1),
%       Tmax = factor * Tmin        (accel 1.50, skidpad 1.25, autocross 1.45)
%   Skidpad squares both ratios. Slower than Tmax scores p_min. Endurance
%   uses the same formula (p_min = 0, Tmax = 1.45 Tmin) on the total time if
%   every lap was run, plus points for the laps completed.
%   Efficiency compares per-lap averages:
%       EF = (Tmin_lap / T_lap) * (CO2min_lap / CO2_lap)
%       points = 100 * (EF - EF_min) / (EF_max - EF_min)
%   EF_max is the best EF in the field. EF_min is the EF of a car at
%   1.45 x Tmin_lap using the reference CO2 (20.02 kg/100 km over the
%   event distance, shared over the laps). A car slower than 1.45 x Tmin_lap,
%   or above the reference CO2, scores 0 and cannot set CO2min.
%
% The car is entered INTO the field. If it beats the fastest real time it
% becomes Tmin and scores the maximum; its CO2 can set CO2min and its EF can
% set EF_max. Every formula here reproduces the published 2026 Michigan EV
% scores (checked by vd_selftest).

R = fsae_rules();
if ~isfield(ev, 'laps') || isempty(ev.laps), ev.laps = F.laps; end
tf = struct('sprint', 1, 'endurance', 1);
if isfield(ev, 'time_factor') && ~isempty(ev.time_factor), tf = ev.time_factor; end

brk = struct('accel', NaN, 'skidpad', NaN, 'autocross', NaN, 'endurance', NaN, ...
             'endurance_time', NaN, 'endurance_laps', NaN, 'efficiency', NaN, ...
             'ef', NaN, 'ef_min', NaN, 'ef_max', NaN, 'co2_lap', NaN);
brk.t_scored = struct('accel', NaN, 'skidpad', NaN, 'autocross', NaN, ...
                      'endurance_lap', NaN, 'endurance_total', NaN);
brk.tmin     = struct();
missing = {};

% --- accel, skidpad, autocross ----------------------------------------------
for e = {'accel', 'skidpad', 'autocross'}
    k = e{1};
    t = get_num(ev, k);
    if isnan(t), missing{end+1} = k; continue; end %#ok<AGROW>
    t    = t * tf.sprint;
    tmin = min([F.(k).t(:); t]);
    brk.(k) = time_points(t, tmin, R.score.(k));
    brk.t_scored.(k) = t;
    brk.tmin.(k)     = tmin;
end

% --- endurance: time (finishers only) + laps ---------------------------------
t_lap = get_num(ev, 'endurance_lap');
if isnan(t_lap)
    missing{end+1} = 'endurance';
else
    t_lap = t_lap * tf.endurance;
    n = floor(ev.laps);
    assert(n >= 0 && n < numel(R.endurance_lap_points), 'fsae_points:laps', ...
           'ev.laps = %g is outside the laps-points table (0-%d).', ...
           ev.laps, numel(R.endurance_lap_points) - 1);
    brk.endurance_laps = R.endurance_lap_points(n + 1);
    if n >= F.laps
        T    = t_lap * F.laps;
        tmin = min([F.endurance.t(:); T]);          % NaN (did not finish) ignored
        brk.endurance_time = time_points(T, tmin, R.score.endurance);
        brk.t_scored.endurance_total = T;
        brk.tmin.endurance = tmin;
    else
        brk.endurance_time = 0;                     % did not finish: laps only
    end
    brk.endurance = brk.endurance_time + brk.endurance_laps;
    brk.t_scored.endurance_lap = t_lap;
end

% --- efficiency --------------------------------------------------------------
% It moves opposite to endurance: a higher endurance power cap is faster
% but uses more energy.
e_lap = get_num(ev, 'energy_lap_kWh');
if isnan(t_lap) || isnan(e_lap)
    missing{end+1} = 'efficiency';
else
    E   = F.efficiency;
    lap = [E.lap_s(:); t_lap];                                  % ours is last
    co2 = [E.co2_lap(:); R.co2_kg_per_kWh * e_lap];             % [kg per lap]
    co2_ref = R.co2_ref_kg_per_km * (R.endurance_m/1000) / F.laps;   % [kg per lap]
    t_lap_min = min(lap);
    ok = lap <= R.ef_min_time_factor * t_lap_min & co2 <= co2_ref;   % NaN -> false
    co2_min = min(co2(ok));
    ef      = (t_lap_min ./ lap) .* (co2_min ./ co2);
    ef_max  = max(ef(ok));
    ef_min  = (1 / R.ef_min_time_factor) * (co2_min / co2_ref);
    if ok(end)
        brk.efficiency = R.efficiency_points * (ef(end) - ef_min) / (ef_max - ef_min);
    else
        brk.efficiency = 0;
    end
    brk.ef = ef(end);  brk.ef_min = ef_min;  brk.ef_max = ef_max;
    brk.co2_lap = co2(end);
    brk.tmin.efficiency_lap = t_lap_min;
    brk.co2_min_lap = co2_min;
end

vals = [brk.accel brk.skidpad brk.autocross brk.endurance brk.efficiency];
total = sum(vals(~isnan(vals)));
brk.missing  = missing;
brk.complete = isempty(missing);
end


function pts = time_points(t, tmin, s)
% Points for time t against the fastest time tmin (s = one fsae_rules().score entry).
tmax = s.t_max_factor * tmin;
if t > tmax
    pts = s.p_min;
elseif s.squared
    pts = s.p_min + s.p_var * ((tmax/t)^2 - 1) / ((tmax/tmin)^2 - 1);
else
    pts = s.p_min + s.p_var * (tmax/t - 1) / (tmax/tmin - 1);
end
end


function v = get_num(ev, f)
if isfield(ev, f) && ~isempty(ev.(f)) && isfinite(ev.(f)), v = ev.(f); else, v = NaN; end
end
