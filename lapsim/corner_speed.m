function vc = corner_speed(p, kappa, ayfun, mode)
% CORNER_SPEED  Highest steady speed [m/s] through curvature kappa [1/m].
%   vc = corner_speed(p, kappa)          uses ay_limit
%   vc = corner_speed(p, kappa, ayfun)   ayfun(v) [g]: a precomputed lateral
%                                        limit (lap_sim passes one)
%   vc = corner_speed(p, kappa, [], 'hold')
%       the highest speed the car can HOLD on that radius (skidpad). Going
%       round at constant speed, the rear tires must also push forward against
%       aero drag, rolling resistance and the tires' own slip drag, and that
%       push uses grip the corner could otherwise use. ax_combined says
%       whether full throttle can still hold the speed.
%
% Solves v^2*|kappa| = ay_max(v)*g by bisection, capped at the rev-limited
% top speed p.v_max. Assumes a single crossing: the cornering demand v^2*kappa
% grows faster with speed than the downforce-assisted grip.
%
% lap_sim uses the plain limit as its speed ceiling; inside a corner its
% forward pass then slows the car to what it can hold.

if nargin < 3 || isempty(ayfun)
    ayfun = @(v) ay_limit(p, v);
end
hold_speed = nargin >= 4 && strcmpi(mode, 'hold');
kappa = abs(kappa);
if kappa < 1e-6, vc = p.v_max; return; end

lo = 0;    hi = p.v_max;             % v = 0 is always feasible
if hi^2*kappa <= ayfun(hi) * p.g     % the corner does not limit below top speed
    vc = hi;
else
    NITER = 30;                      % resolves v to well under 1e-6 m/s
    for k = 1:NITER
        mid = 0.5*(lo + hi);
        if mid^2*kappa <= ayfun(mid) * p.g
            lo = mid;
        else
            hi = mid;
        end
    end
    vc = lo;
end

if hold_speed
    holds = @(v) ax_combined(p, v, v^2*kappa/p.g, 'accel') >= 0;
    if ~holds(vc)
        lo = 0;  hi = vc;
        for k = 1:30
            mid = 0.5*(lo + hi);
            if holds(mid), lo = mid; else, hi = mid; end
        end
        vc = lo;
    end
end
end
