function [D, info] = slip_drag(p, v, ay_g)
% SLIP_DRAG  Drag from tire slip angles in steady cornering [N].
%   D = slip_drag(p, v, ay_g)           v [m/s], ay_g lateral acceleration [g]
%   [D, info] = slip_drag(p, v, ay_g)   info.alpha_f_deg, info.alpha_r_deg,
%                                       info.saturated, info.on (the switch)
% v and ay_g may be arrays of the same size (or one of them scalar); every
% output then has that size. A whole lap is one call.
%
% WHAT IT IS
% A tire makes lateral force only by running at a slip angle alpha: the wheel
% points slightly inside the direction it travels, so the tread is dragged
% sideways at v*sin(alpha) as it rolls. Sliding against friction turns energy
% into heat at a rate Fy * v*sin(alpha). Seen as a force: the tire force acts
% at right angles to the wheel, which moves at alpha to its own plane, so the
% force is tilted back by alpha from the direction of travel. Fy*cos(alpha)
% turns the car; Fy*sin(alpha) holds it back. For the car,
%     D = Fy_f*sin(alpha_f) + Fy_r*sin(alpha_r)
% In the linear range alpha = Fy/C_alpha, so D ~ Fy^2/C_alpha: the drag grows
% with the square of the cornering force. Near the limit it grows much faster,
% because the last few percent of grip cost a large slip angle.
%
% METHOD (the same steady-state construction as axle_grip)
%   1. Lateral force per axle from the yaw-moment balance: front m*ay*b/L,
%      rear m*ay*a/L.
%   2. Tire loads at speed v: axle loads from road_loads, lateral transfer
%      split by LLTD (an inner wheel off the ground leaves the axle on the
%      outer one, as in axle_grip); per-corner camber from p.camber_deg.
%   3. Both tires on an axle run at the same slip angle (no toe, no Ackermann).
%      The axle force Fy_out(alpha) + Fy_in(alpha) comes from tire_forces;
%      alpha is the smallest slip angle that delivers the axle's share.
%   4. The two tires peak at different slip angles, so the pair cannot quite
%      reach the sum of their peaks that axle_grip allows. If the share is
%      above what the pair can give, alpha is taken where the pair peaks and
%      info.saturated is true.
%
% The energy form Fy * (lateral slip speed) is the standard definition of
% lateral tire slip power loss (e.g. Chatzikomis et al., Mech. Syst. Signal
% Process. 128, 2019, eq. 20).
%
% Returns 0 on a straight (ay = 0) and when p.slip_drag is 'off'.
%
% Not modelled: longitudinal slip loss (drive and brake force times wheel
% slip speed), combined slip (driving or braking in a corner needs more
% slip), toe and Ackermann scrub, the cos(steer) factor, any change of
% rolling resistance with slip angle.

on = slip_drag_on(p);

% Expand v and ay_g to a common size; work on columns.
sz = size(v + ay_g);
v  = v(:)    + zeros(prod(sz), 1);
ay_g = ay_g(:) + zeros(prod(sz), 1);
n  = numel(v);
D  = zeros(n, 1);  a_f = zeros(n, 1);  a_r = zeros(n, 1);  sat = false(n, 1);

act = find(ay_g > 0);
if on && ~isempty(act)
    ALPHA = 0:0.25:30;                   % slip-angle grid [deg]
    m  = numel(act);
    ay = ay_g(act) * p.g;
    R  = road_loads(p, v(act));
    dF_f = p.LLTD       * p.m * ay * p.h_cg / p.t_f;
    dF_r = (1 - p.LLTD) * p.m * ay * p.h_cg / p.t_r;
    Fz = [R.Nf/2 + dF_f, R.Nf/2 - dF_f, R.Nr/2 + dF_r, R.Nr/2 - dF_r];  % fo fi ro ri [N]
    lift_f = Fz(:,2) <= 0;   Fz(lift_f, 1) = R.Nf(lift_f);   % outer carries the axle
    lift_r = Fz(:,4) <= 0;   Fz(lift_r, 3) = R.Nr(lift_r);
    lifted = Fz <= 0;
    Fz(lifted) = 1;                      % placeholder load; its force is zeroed below

    g = [0 0 0 0];
    if isfield(p, 'camber_deg') && ~isempty(p.camber_deg)
        c = {'fo', 'fi', 'ro', 'ri'};
        for k = 1:4
            if isfield(p.camber_deg, c{k}), g(k) = p.camber_deg.(c{k}); end
        end
    end
    G = repmat(g, m, 1);

    % Loads outside the tire data are handled (and reported) by the grip
    % solvers; do not repeat those warnings for every cornering point here.
    w1 = warning('off', 'mu_of_load:beyondDonorCoverage');
    w2 = warning('off', 'mu_of_load:belowFloor');
    restore1 = onCleanup(@() warning(w1.state, w1.identifier)); %#ok<NASGU>
    restore2 = onCleanup(@() warning(w2.state, w2.identifier)); %#ok<NASGU>

    % All 4*m tires in one call: rows fo(1..m), fi, ro, ri; columns ALPHA.
    T  = tire_forces(p, Fz(:) / p.N_PER_LBF, ALPHA, G(:));
    Fy = T.Fy_lbf * p.N_PER_LBF;         % [N]
    Fy(lifted(:), :) = 0;
    rows = @(k) (k-1)*m + (1:m);

    Fy_f = p.m * ay * p.b / p.L;         % axle shares from the yaw balance [N]
    Fy_r = p.m * ay * p.a / p.L;
    [a_f(act), sat_f] = slip_for(Fy(rows(1),:) + Fy(rows(2),:), Fy_f, ALPHA);
    [a_r(act), sat_r] = slip_for(Fy(rows(3),:) + Fy(rows(4),:), Fy_r, ALPHA);
    sat(act) = sat_f | sat_r;
    D(act) = Fy_f .* sind(a_f(act)) + Fy_r .* sind(a_r(act));
end

D = reshape(D, sz);
info = struct('alpha_f_deg', reshape(a_f, sz), 'alpha_r_deg', reshape(a_r, sz), ...
              'saturated', reshape(sat, sz), 'on', on);
end


function [a, saturated] = slip_for(F_axle, F_need, ALPHA)
% Smallest slip angle at which each row of the axle curve reaches F_need
% (linear interpolation between grid points). Past the curve's peak the
% force falls; only the rising side counts.
A    = ALPHA(:);
F_up = cummax(F_axle, 2);
i    = sum(F_up < F_need, 2) + 1;        % first grid point at or above the need
saturated = i > numel(A);
a = zeros(size(F_need));
[~, ipk] = max(F_axle, [], 2);
a(saturated) = A(ipk(saturated));        % beyond the pair's peak: sit at the peak
r  = find(~saturated & i > 1);           % i == 1 means no slip needed (alpha 0)
lo = sub2ind(size(F_up), r, i(r) - 1);
hi = sub2ind(size(F_up), r, i(r));
a(r) = A(i(r) - 1) + (F_need(r) - F_up(lo)) ./ (F_up(hi) - F_up(lo)) .* (A(i(r)) - A(i(r) - 1));
end


function on = slip_drag_on(p)
% p.slip_drag: 'on' (default) or 'off'.
on = true;
if isfield(p, 'slip_drag') && ~isempty(p.slip_drag)
    switch lower(p.slip_drag)
        case 'on',  on = true;
        case 'off', on = false;
        otherwise
            error('slip_drag:badSwitch', ...
                  'p.slip_drag = ''%s'' not recognized - use ''on'' or ''off''.', p.slip_drag);
    end
end
end
