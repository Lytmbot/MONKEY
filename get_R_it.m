function [ R_it ] = get_R_it( k, theta )
%% IN:
%   - k a rotational constant
%   - current time to solve R_0t(t) for


%% OUT:
%   - R_0t, the orientation @ time

%% ********************************* GET R_it *****************************
e = [k(1)*sind(theta/2) , k(2)*sind(theta/2) , k(3)*sind(theta/2) , cosd(theta/2)];

R_it = [ 1-2*(e(2)^2+e(3)^2)   , 2*(e(1)*e(2)-e(3)*e(4)) , 2*(e(1)*e(3)+e(2)*e(4)) ;
       2*(e(1)*e(2)+e(3)*e(4)) , 1-2*(e(1)^2+e(3)^2)     , 2*(e(2)*e(3)-e(1)*e(4)) ;
       2*(e(1)*e(3)-e(2)*e(4)) , 2*(e(2)*e(3)+e(1)*e(4)) , 1-2*(e(1)^2+e(2)^2)    ]; 
  
  
   
end




