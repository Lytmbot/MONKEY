function [ Jacobian ] = get_JACOBIAN_derivative( T07)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% ** TEST INPUT
D = [2,5,5,2];

L01 = D(1);
L23 = D(2);
L45 = D(3);
L67 = D(4);

qs = I_KINEMATICS_getangles(T07);
s1 = sind(qs(1));
c1 = cosd(qs(1));
s2 = sind(qs(2));
c2 = cosd(qs(2));
s3 = sind(qs(3));
c3 = cosd(qs(3));
s4 = sind(qs(4));
c4 = cosd(qs(4));
s5 = sind(qs(5));
c5 = cosd(qs(5));
s6 = sind(qs(6));
c6 = cosd(qs(6));



row = 1;
col = 1;

% d/dq1(Ex) 
Jacobian(row,col)=-L45*c4*s1*s3 + L45*c4*c3*c1*s2 + L45*c2*c1*s4 - L67*s6*c4*s1*s3 + L67*s6*c4*c3*c1*s2 + L67*s6*c2*c1*s4 + L67*c6*s5*s1*c3 - L67*c6*s5*c1*s2*s3 + L67*c6*c5*s4*s1*s3 + L67*c6*c5*s4*c3*c1*s2 - L67*c6*c5*c2*c4*c1 - L23*c2*c1;
% d/dq1(Ey) 
Jacobian(row+1,col) = L45*c4*c1*s3 + L45*c4*s1*c3*s2 + L45*s1*c2*s4 + L67*s6*c4*c1*s3 + L67*s6*c4*s1*c3*s2 + L67*s6*s1*c2*s4 - L67*c6*s5*c3*c1 - L67*c6*s5*s1*s2*s3 - L67*c6*c5*s4*c1*s3 + L67*c6*c5*s4*s1*c3*s2 - L67*c6*c5*s1*c2*c4 - L23*s1*c2;
% d/dq1(Ez) 
Jacobian(row+2,col) = 0;

col = col + 1;
% d/dq2(Ex)
Jacobian(row,col)= L45*c4*c3*s1*c2 - L45*s2*s1*s4 + L67*s6*c4*c3*s1*c2 - L67*s6*s2*s1*s4 - L67*c6*s5*s1*c2*s3 + L67*c6*c5*s4*c3*s1*c2 + L67*c6*c5*s2*c4*s1 + L23*s2*s1;
% d/dq2(Ey) 
Jacobian(row+1,col)=-L45*c4*c1*c3*c2 + L45*c1*s2*s4 - L67*s6*c4*c1*c3*c2 + L67*s6*c1*s2*s4 + L67*c6*s5*c1*c2*s3 - L67*c6*c5*s4*c1*c3*c2 - L67*c6*c5*c1*s2*c4 - L23*c1*s2;
% d/dq2(Ez) 
Jacobian(row+2,col)= L23*c2 + L67*c6*c5*c4*c2 - L67*c6*c5*s2*c3*s4 - L67*c6*s2*s3*s5 - L67*s6*c2*s4 + L67*s6*s2*c3*c4 - L45*c2*s4 + L45*s2*c3*c4;

col = col + 1;
% d/dq3(Ex) 
Jacobian(row,col)= L45*c4*c1*c3 - L45*c4*s3*s1*s2 + L67*s6*c4*c1*c3 + L67*s6*c4*c3*c1*s2 + L67*c6*s5*c1*s3 - L67*c6*s5*s1*s2*c3 - L67*c6*c5*s4*c1*c3 - L67*c6*c5*s4*s3*s1*s2;
% d/dq3(Ey) 
Jacobian(row+1,col)= L45*c4*s1*c3 + L45*c4*c1*s3*s2 + L67*s6*c4*s1*c3 + L67*s6*c4*c1*s3*s2 + L67*c6*s5*s3*s1 + L67*c6*s5*c1*s2*c3 - L67*c6*c5*s4*s1*c3 + L67*c6*c5*s4*c1*s3*s2;
% d/dq3(Ez) 
Jacobian(row+2,col)=-L67*c6*c5*c2*s3*s4 + L67*c6*c2*c3*s5 + L67*s6*c2*s3*c4 + L45*c2*s3*c4;
  
col = col + 1;
% d/dq4(Ex) 
Jacobian(row,col)=-L45*s4*c1*s3 - L45*s4*c3*s1*s2 + L45*c2*s1*c4 - L67*s6*s4*c1*s3 - L67*s6*s4*c3*s1*s2 + L67*s6*c2*s1*c4 - L67*c6*c5*c4*c1*s3 + L67*c6*c5*c4*c3*s1*s2 + L67*c6*c5*c2*s4*s1;
% d/dq4(Ey) 
Jacobian(row+1,col)=-L45*s4*s1*s3 + L45*s4*c1*c3*s2 - L45*c1*c2*c4 - L67*s6*s4*s1*s3 + L67*s6*s4*c1*c3*s2 - L67*s6*c1*c2*c4 - L67*c6*c5*c4*s1*s3 - L67*c6*c5*c4*c1*c3*s2 - L67*c6*c5*c1*c2*s4;
% d/dq4(Ez) 
Jacobian(row+2,col)=-L67*c6*c5*s4*s2 + L67*c6*c5*c2*c3*c4 - L67*s6*s2*c4 + L67*s6*c2*c3*s4 - L45*s2*c4 + L45*c2*c3*s4;
  
col = col + 1;
% d/dq5(Ex) 
Jacobian(row,col)= L67*c6*c5*c1*c3 - L67*c6*c5*s1*s2*s3 + L67*c6*s5*s4*c1*s3 - L67*c6*s5*s4*c3*s1*s2 + L67*c6*s5*c2*c4*s1;
% d/dq5(Ey) 
Jacobian(row+1,col)= L67*c6*c5*c3*s1 + L67*c6*c5*c1*s2*s3 + L67*c6*s5*s4*s1*s3 + L67*c6*s5*s4*c1*c3*s2 - L67*c6*s5*c1*c2*c4;
% d/dq5(Ez) 
Jacobian(row+2,col)=-L67*c6*s5*c4*s2 - L67*c6*s5*c2*c3*s4 + L67*c6*c2*s3*c5;
  
col = col + 1;
% d/dq6(Ex) 
Jacobian(row,col)= L67*c6*c4*c1*s3 - L67*s6*s4*c3*s1*s2 + L67*c6*c2*s1*s4 + L67*s6*s5*c1*c3 + L67*s6*s5*s1*s2*s3 + L67*s6*c5*s4*c1*s3 - L67*s6*c5*s4*c3*s1*s2 + L67*s6*c5*c2*c4*s1;
% d/dq6(Ey) 
Jacobian(row+1,col)= L67*c6*c4*s1*s3 - L67*c6*c4*c1*c3*s2 - L67*c6*c1*c2*s4 + L67*s6*s5*c3*s1 - L67*s6*s5*c1*s2*s3 + L67*s6*c5*s4*s1*s3 + L67*s6*c5*s4*c1*c3*s2 - L67*s6*c5*c1*c2*c4;
% d/dq6(Ez) 
Jacobian(row+2,col)=-L67*s6*c5*c4*s2 - L67*s6*c5*c2*c3*s4 - L67*s6*c2*s3*s5 - L67*c6*s2*s4 - L67*c6*c2*c3*c4;
  
col = col + 1;
% d/dq7(Ex) 
Jacobian(row,col)= 0;
% d/dq7(Ey) 
Jacobian(row+1,col)= 0;
% d/dq7(Ez) 
Jacobian(row+2,col)= 0;

for i = 1 : 7
    Jacobian(6,i) = 1;
end
