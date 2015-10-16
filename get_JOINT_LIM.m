function [ lim ] = get_JOINT_LIM( joint )
%IN:
%   JOINT TO LIMIT
%OUT:
%   LIMITS

limits(1) = 360;
limits(2) = 180;
limits(3) = 360;
limits(4) = 180;
limits(5) = 360;
limits(6) = 180;
limits(7) = 360;

lim = limits(joint);

end

