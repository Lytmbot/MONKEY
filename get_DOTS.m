function [ q_dot ] = get_DOTS( T07, v )
% I N : 
%       - T07 the rotation and translation of point E (4x4)
%       - v the velocity of point E(x,y,z): 
%                                       linear[x_dot,y_dot,z_dot] 
%                                       angular[w_x,w_y,w_z]
%       - obv zeros for w will maintain orientation.
% O U T :
%       - angular velocity for motors q_dot 1-7

J = get_JACOBIAN(T07);
% disp((J*J'))
% disp(transpose(J))
J_hash =  pinv(J);%J' * inv(J*J') ;

q_dot = J_hash * v;



end

