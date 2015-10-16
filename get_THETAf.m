function [ theta_final ] = get_THETAf( R_0i , R_0f)
%% IN 
%   -initial and final orientation

%% OUT:
%   - theta_f the final value of theta(t)

%% ******************************** GET R_if *******************************

R_if = R_0i\R_0f;
theta_final = 2*acosd( 0.5*sqrt(R_if(1,1)+R_if(2,2)+R_if(3,3)+1) );
