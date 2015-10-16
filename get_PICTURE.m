function [ points_to_draw ] = get_PICTURE(centre, number_of_points, plane)
% Generates *number_of_points* on the circle in *plane* with *center*

%% T H E   C I R C L E
radius = 3;
points_to_draw = zeros(3,number_of_points);
xy_points = circle(centre, radius, number_of_points, plane);

for i = 1 : 1 : number_of_points+1
    
    points_to_draw(plane(1),i) = xy_points(1,i);
    points_to_draw(plane(2),i) = xy_points(2,i);
    points_to_draw(plane(3),i) = centre(plane(3));
        
end
