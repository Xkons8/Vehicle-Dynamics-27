function [ok, locks] = brake_split(p, F_b, F_f, F_r)
%BRAKE_SPLIT Determines whether the brakes can deliver F_b with brake bias
%given
%   F_b is the braking force that the brakes must supply at the tires
%   F_f and F_r are the most force the front/rear tires can handle without locking up


if 0 > F_b
    F_b = 0;
end

locks = '';
if strcmp(p.brake_model, 'ideal')
    ok = F_b <= F_f + F_r;       % Current model used now
    if ~ok
        locks = 'both'; 
    end
elseif strcmp(p.brake_model, 'fixed')
    brake_bias = p.brake_bias_f;
    if brake_bias > 1 || brake_bias < 0 
        error('brake split must be in [0,1]');
    end
    Ff = brake_bias*F_b;
    Fr = (1-brake_bias)*F_b;
    ok = ~(Ff > F_f) && ~(Fr > F_r);
    if ~ok
        if Ff/F_f > Fr/F_r 
            locks = 'front';
        else 
            locks = 'rear';
        end
    end
else
    error('Model must be either "ideal" or "fixed"');
end






end