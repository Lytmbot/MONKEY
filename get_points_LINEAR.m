function [ point ] = get_points_LINEAR( A,time)
%% IN:
%   -time
%   -'A' is a 2x1 matrix containint A0 and A1

%% OUT
%   -the point at time

%%
point = A(1) + A(2)*time;

end

