function [ k ] = get_K( R_0i , R_0f)
%% IN 
%   -initial and final orientation

%% OUT:
%   - k constant to caluculate orientation as a function of theta(t)

%% ******************************** GET R_if *******************************
R_if = transpose(R_0i)*R_0f;
theta = 2*acosd(0.5*sqrt(R_if(1,1)+R_if(2,2)+R_if(3,3)+1));

%% ******************************** GET K *********************************
k = (1/(2*sind(theta)))*[R_if(3,2)-R_if(2,3) , R_if(1,3)-R_if(3,1) , R_if(2,1)-R_if(1,2)];

