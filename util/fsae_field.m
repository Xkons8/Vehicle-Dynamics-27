function F = fsae_field(name)
% FSAE_FIELD  Load a competition's published dynamic-event results (the "field").
%   F = fsae_field('fsae_mi_2026_ev')    competitions/fsae_mi_2026_ev/
%   F = fsae_field()                     the competition named in p.competition
%
% fsae_points scores a car against these real results: the fastest time in
% each event, the best efficiency, and so on, come from here.
%
% The folder holds one CSV per event, copied from the official results.
% Lines starting with # are notes; a blank cell means "none" (NaN).
%   accel.csv, skidpad.csv, autocross.csv
%                   car, team, best_time_s, score
%   endurance.csv   car, team, laps, adjusted_time_s, time_score, laps_score
%                   (adjusted_time_s blank for a team that did not finish)
%   efficiency.csv  car, team, avg_lap_s, laps, energy_kWh, co2_kg_per_lap, ..., score
%                   (energy blank for a team with no efficiency result;
%                   co2_kg_per_lap optional, else 0.65 kg/kWh x energy / laps)
% The published scores are kept so vd_selftest can check fsae_points
% against them. To score against another event, add a folder with the same
% five files and set p.competition in vehicle_params.m.
%
% F.name, F.laps (laps in the endurance event: the most any team completed)
% F.accel / .skidpad / .autocross   .team, .t [s], .score
% F.endurance    .team, .laps, .t [s, total], .time_score, .laps_score
% F.efficiency   .team, .lap_s [s, average lap], .laps, .energy_kWh,
%                .co2_lap [kg per lap], .score

if nargin < 1 || isempty(name)
    p = vehicle_params();
    name = p.competition;
end
d = fullfile(vd_root(), 'competitions', name);
if ~exist(d, 'dir')
    error('fsae_field:noCompetition', ...
          'No folder competitions/%s. Available: %s', name, available());
end

F.name = name;
for ev = {'accel', 'skidpad', 'autocross'}
    T = read_csv(fullfile(d, [ev{1} '.csv']), {'team', 'best_time_s', 'score'});
    F.(ev{1}) = struct('team', {T.team}, 't', T.best_time_s, 'score', T.score);
end

T = read_csv(fullfile(d, 'endurance.csv'), ...
             {'team', 'laps', 'adjusted_time_s', 'time_score', 'laps_score'});
F.endurance = struct('team', {T.team}, 'laps', T.laps, 't', T.adjusted_time_s, ...
                     'time_score', T.time_score, 'laps_score', T.laps_score);
F.laps = max(T.laps);

T = read_csv(fullfile(d, 'efficiency.csv'), ...
             {'team', 'avg_lap_s', 'laps', 'energy_kWh', 'score'});
% CO2 per lap = 0.65 kg/kWh x energy / laps. Where the officials published a
% per-lap value that differs by more than its rounding (0.0001 kg), theirs
% wins: in 2026 two teams' published value is their CO2 over 23 laps.
co2_lap = fsae_rules().co2_kg_per_kWh * T.energy_kWh ./ T.laps;
if isfield(T, 'co2_kg_per_lap')
    differs = abs(T.co2_kg_per_lap - co2_lap) > 1e-4;      % NaN -> false
    co2_lap(differs) = T.co2_kg_per_lap(differs);
end
F.efficiency = struct('team', {T.team}, 'lap_s', T.avg_lap_s, 'laps', T.laps, ...
                      'energy_kWh', T.energy_kWh, 'co2_lap', co2_lap, 'score', T.score);
end


function T = read_csv(fname, need)
% Columns by header name: 'team' stays text, every other column is numeric.
fid = fopen(fname, 'r');
if fid < 0
    error('fsae_field:missingFile', 'Missing %s', fname);
end
closer = onCleanup(@() fclose(fid));

hdr = {};
rows = {};
while true
    ln = fgetl(fid);
    if ~ischar(ln), break; end
    ln = strtrim(ln);
    if isempty(ln) || ln(1) == '#', continue; end
    cells = strtrim(regexp(ln, ',', 'split'));   % keeps empty cells
    if isempty(hdr)
        hdr  = cells;
        rows = cell(0, numel(hdr));
        continue
    end
    if numel(cells) ~= numel(hdr)
        error('fsae_field:badRow', '%s: "%s" has %d cells, the header has %d.', ...
              fname, ln, numel(cells), numel(hdr));
    end
    rows(end+1, :) = cells; %#ok<AGROW>
end

lost = need(~ismember(need, hdr));
if ~isempty(lost)
    error('fsae_field:badHeader', '%s has no column(s): %s', fname, strjoin(lost, ', '));
end
T = struct();
for j = 1:numel(hdr)
    if strcmp(hdr{j}, 'team')
        T.team = rows(:, j);
    else
        T.(hdr{j}) = str2double(rows(:, j));
    end
end
end


function s = available()
d = dir(fullfile(vd_root(), 'competitions'));
d = d([d.isdir] & ~startsWith({d.name}, '.'));
if isempty(d), s = '(none)'; else, s = strjoin({d.name}, ', '); end
end
