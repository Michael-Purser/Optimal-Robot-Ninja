function vec2 = interpolateUntil(vec,N)
% Function that takes a vector 'vec' and returns that vector
% + interpolated values until it has length N.

% number of points to be added in each interval:
n = length(vec);
na = ceil((N-n)/(n-1));

% add points until vector has length N:
vec2 = [];
for i = 1:(n-1)
    vec2 = [vec2;vec(i)];
    for j = 1:na
        if length(vec2)<(N-(n-i))
            p_j = vec(i)+j*(vec(i+1)-vec(i))/(na+1);
            vec2 = [vec2;p_j];
        end
    end
end
vec2(end+1) = vec(end);