function [ picture ] = get_LINE( pt_1, pt_2, num_points )
%return a series of points that can be used to draw a cube 

picture = zeros(3,num_points);

x_span = pt_2(1)-pt_1(1);
y_span = pt_2(2)-pt_1(2);
z_span = pt_2(3)-pt_1(3);

for i = 1 : num_points+1
    
    picture(1,i) = pt_1(1) + (x_span/num_points)*(i-1) ;
    picture(2,i) = pt_1(2) + (y_span/num_points)*(i-1) ;
    picture(3,i) = pt_1(3) + (z_span/num_points)*(i-1) ;
    
end
end

