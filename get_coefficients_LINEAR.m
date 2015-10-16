function [ A ] = get_coefficients_LINEAR( P1, P2, time_end )
% I N:
%   -P1 the start of the trajectory [x1,y1,z1]
%   -P2 the end of the trejectory [x2,y2,z2]
%
% O U T:
%   - constant coefficients that define the path from P1 -> P2 in time
%   - a 2x3 matrix that handles each variable (x,y,z) seperatly
%   - x[Ao,A1]
%   - y[Ao,A1]
%   - z[Ao,A1]


% Solve for F(t) = A0 + A1(t)
% no velocity!!!
% three seperate equations
% Fx(t) = ...
% Fy(t) = ...
% Fz(t) = ...

x=1;
y=2;
z=3;
A0 = 1;
A1 = 2;

A = zeros(3,2);

%% GET A0
%for x
A(x,A0) = P1(x);
%for y
A(y,A0) = P1(y);
%for z
A(z,A0) = P1(z);


%% GET A1
%for x
A(x,A1)= (P2(x) - A(x,A0))/time_end; 
%for y
A(y,A1)= (P2(y) - A(y,A0))/time_end; 
%for z
A(z,A1)= (P2(z) - A(z,A0))/time_end; 


end

