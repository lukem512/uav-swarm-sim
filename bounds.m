%
% Bounds-checking functions, dependent upon timestep
%
% Luke Mitchell, Jan 2016
%
function b = bounds
    b.minv  = @minv;
    b.maxv  = @maxv;
    b.minmu = @minmu;
    b.maxmu = @maxmu;
end

% Minimum speed.
function v = minv(dt)
    v = 10 * dt;
end

% Max speed!
function v = maxv(dt)
    v = 20 * dt;
end

% Minimum turning.
function mu = minmu(dt)
    mu = -6 * dt;
end

% Max turning!
function mu = maxmu(dt)
    mu = 6 * dt;
end