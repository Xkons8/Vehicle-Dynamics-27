function out = run_aero_targets(p)
% RUN_AERO_TARGETS  Downforce, drag and L/D targets in FSAE points.
% Sweeps added downforce along package L/D lines (drag and device mass grow
% with ClA), simulates the dynamic events for each (fsae_events) and scores
% them against a real competition (fsae_points, p.competition).
%
%   out = run_aero_targets()      the active car (vd_car / cars/config_<CAR>.m)
%   out = run_aero_targets(p)     an explicit params struct; build "what if?" cars with vd_set

if nargin < 1 || isempty(p), p = vehicle_params(); end
vd_warn('reset');
here = vd_root();

% Sweep and package assumptions
DCLA        = 0:0.75:3.75;   % added downforce [m^2]
LD_PKG      = [3 4 5];       % whole-package lift/drag lines [-]
DM_PER_CLA  = 4.0;           % aero package mass penalty [kg per m^2 ClA] PROVISIONAL
CLA_TARGET  = p.scenario.ClA_target;   % target downforce [m^2]
CLA_BAND    = [3.5 4.5];     % target range [m^2]
LD_TARGET   = 4.0;           % package L/D assumed at the target [-] CHOICE

ctx = struct('p', p, 'F', fsae_field(p.competition));

% Baseline and sweep
[pts0, d0] = score_point(ctx, 0, inf, 0);
P = nan(numel(LD_PKG), numel(DCLA));
for i = 1:numel(LD_PKG)
    for j = 1:numel(DCLA)
        P(i,j) = score_point(ctx, DCLA(j), LD_PKG(i), DM_PER_CLA);
    end
end

% Exchange rates at the target (forward differences)
dc_t = CLA_TARGET - p.ClA;
[pts_t, d_t] = score_point(ctx, dc_t, LD_TARGET, DM_PER_CLA);
xr_cla = (score_point_abs(ctx, dc_t+0.4, d_t.CdA,     DM_PER_CLA*dc_t) - pts_t) / 0.4;
xr_cda = (score_point_abs(ctx, dc_t,     d_t.CdA+0.2, DM_PER_CLA*dc_t) - pts_t) / 0.2;
xr_m   = (score_point_abs(ctx, dc_t,     d_t.CdA,     DM_PER_CLA*dc_t+10) - pts_t) / 10;
ld_floor_drag = abs(xr_cda) / xr_cla;                       % drag-only
ld_floor_full = abs(xr_cda) / (xr_cla + xr_m*DM_PER_CLA);   % incl. device mass

fprintf('\nAero - %s  (dynamic-event points against the %s results)\n', p.car, p.competition);
vd_row('Points, this car', sprintf('%.0f', pts0), sprintf('ClA %.2f, CdA %.2f', p.ClA, p.CdA));
fprintf('    accel %.1f, skidpad %.1f, autocross %.1f, endurance %.1f, efficiency %.1f\n', ...
        d0.points.accel, d0.points.skidpad, d0.points.autocross, d0.points.endurance, ...
        d0.points.efficiency);
fprintf('    autocross %.1f s, endurance %.0f s using %.2f kWh at a %.1f kW power cap\n', ...
        d0.t_ax, d0.t_tot, d0.E_kWh, d0.cap/1e3);

fprintf('\n  Points by added downforce (columns) and whole-package lift/drag (rows)\n');
fprintf('    %-12s', 'added ClA'); fprintf(' %7.2f', DCLA); fprintf('   [m^2]\n');
for i = 1:numel(LD_PKG)
    fprintf('    %-12s', sprintf('L/D %.0f', LD_PKG(i))); fprintf(' %7.0f', P(i,:)); fprintf('\n');
end
if all(diff(P(end,:)) > 0)
    fprintf('  Points still rise at the right-hand edge on the best L/D line.\n');
end
fprintf('\n');
vd_row('Downforce target', sprintf('ClA %.1f m^2', CLA_TARGET), ...
       sprintf('range %.1f-%.1f', CLA_BAND));
vd_row('Drag budget at the target', sprintf('CdA %.2f m^2', d_t.CdA), ...
       sprintf('package L/D %.1f', LD_TARGET));
vd_row('Points at the target', sprintf('%.0f', pts_t), sprintf('%+.0f vs this car', pts_t - pts0));
vd_row('  power cap / energy at the target', ...
       sprintf('%.1f kW', d_t.cap/1e3), sprintf('%.2f kWh', d_t.E_kWh));
vd_row('Points per +1 m^2 ClA', sprintf('%+.1f', xr_cla));
vd_row('Points per +1 m^2 CdA', sprintf('%+.1f', xr_cda));
vd_row('Points per +1 kg', sprintf('%+.2f', xr_m));
vd_row('Lowest device L/D worth adding', sprintf('%.1f', ld_floor_full), ...
       sprintf('%.1f ignoring mass', ld_floor_drag));
fprintf(['Assumes: lap simulation on the axle models; downforce balance fixed (see\n' ...
         '         run_balance_targets); simulated times x%.2f (accel, skidpad, autocross)\n' ...
         '         and x%.2f (endurance) for the driver, not calibrated; device mass\n' ...
         '         %.0f kg per m^2 ClA; package L/D %.1f at the target.\n'], ...
        p.scenario.time_factor_sprint, p.scenario.time_factor_endurance, DM_PER_CLA, LD_TARGET);

out = struct('pts_baseline', pts0, 'pts_target', pts_t, 'dcla', DCLA, ...
             'ld_pkg', LD_PKG, 'points', P, 'ClA_target', CLA_TARGET, ...
             'CdA_budget', d_t.CdA, 'xr_cla', xr_cla, 'xr_cda', xr_cda, ...
             'xr_m', xr_m, 'ld_floor_full', ld_floor_full, ...
             'ld_floor_drag', ld_floor_drag, 'target_detail', d_t);

if vd_plots()
try
    make_plot(p, DCLA, LD_PKG, P, pts0, CLA_TARGET, CLA_BAND, pts_t, here);
    fprintf('Saved plots/aero_targets.png\n');
catch e
    fprintf('Plot not saved: %s\n', e.message);
end
end
end

function [pts, d] = score_point(ctx, dcla, ld_pkg, dm_per_cla)
% A point on a package line: drag and device mass follow the added downforce.
CdA = ctx.p.CdA + dcla / ld_pkg;            % inf -> baseline drag
[pts, d] = score_point_abs(ctx, dcla, CdA, dm_per_cla * dcla);
end

function [pts, d] = score_point_abs(ctx, dcla, CdA, dm)
% dm is aero hardware on the car, so it goes on m_car (vd_set re-derives the rest).
p2 = vd_set(ctx.p, 'ClA', ctx.p.ClA + dcla, 'CdA', CdA, 'm_car', ctx.p.m_car + dm);
ev = fsae_events(p2);
[pts, brk] = fsae_points(ev, ctx.F);

% Report the times that were scored (after the time factor).
d = struct('t_ac', brk.t_scored.accel, 't_sk', brk.t_scored.skidpad, ...
           't_ax', brk.t_scored.autocross, 't_en', ev.endurance_lap, ...
           't_tot', brk.t_scored.endurance_total, ...
           'E_kWh', ev.energy_lap_kWh * ctx.F.laps, 'cap', ev.cap_W, 'CdA', p2.CdA);
d.points = struct('accel', brk.accel, 'skidpad', brk.skidpad, 'autocross', brk.autocross, ...
                  'endurance', brk.endurance, 'efficiency', brk.efficiency);
end

function make_plot(p, DCLA, LD_PKG, P, pts0, cla_t, xr, pts_t, here)
f = figure('Visible', 'off', 'Position', [80 80 900 520], 'Color', 'w');
hold on;
for i = 1:numel(LD_PKG)
    plot(p.ClA + DCLA, P(i,:), '-o', 'LineWidth', 1.8, ...
         'DisplayName', sprintf('package L/D = %.0f', LD_PKG(i)));
end
xline(p.ClA, ':', 'this car', 'LineWidth', 1.2, 'HandleVisibility', 'off');
fill([xr(1) xr(2) xr(2) xr(1)], [min(P(:))-5 min(P(:))-5 max(P(:))+5 max(P(:))+5], ...
     [0.13 0.40 0.67], 'FaceAlpha', 0.08, 'EdgeColor', 'none', 'DisplayName', 'target range');
plot(cla_t, pts_t, 'p', 'MarkerSize', 16, 'MarkerFaceColor', [0.9 0.65 0], ...
     'MarkerEdgeColor', 'k', 'DisplayName', 'target');
yline(pts0, '--', 'baseline', 'LineWidth', 1.0, 'HandleVisibility', 'off');
xlabel('total ClA [m^2]');  ylabel('projected dynamic points');
title(sprintf('Points vs downforce along package L/D lines (%s)', p.competition), ...
      'FontWeight', 'bold');
legend('Location', 'southeast');  grid on;
outdir = fullfile(here, 'plots');
if ~exist(outdir, 'dir'), mkdir(outdir); end
saveas(f, fullfile(outdir, 'aero_targets.png'));
close(f);
end
