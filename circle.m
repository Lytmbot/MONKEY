function [points] = circle(centre,r,num_points,plane)
%%********************  W H A T   D O E S   I T   D O  ********************
%I N:
% centre... coordinates of the center of the circle
% r... the radius of the circle
% num_points.... number of points to generate
% plane, xy,xz,yx plane to draw in

xy = [1,2,3]; % draw x y plane
xz = [1,3,2];
yz = [2,3,1];

%get points that make up the circle
points = zeros(2,num_points);
ang = 0;
step_size = 2*pi / num_points;

% map point into 3d space
for i=1:1:num_points+1;
    xp=r*cos(ang);
    yp=r*sin(ang);
    if plane == xy   
        points(1,i) = xp + centre(1); 
        points(2,i) = yp + centre(2);
    end
    
    if plane == xz
        points(1,i) = xp + centre(1);
        points(2,i) = yp + centre(3);
    end
    
    if plane == yz   
        points(1,i) = xp + centre(2); 
        points(2,i) = yp + centre(3);
    end
    ang = ang+step_size;
end

end