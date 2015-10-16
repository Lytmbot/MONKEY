%% GET THE JACOBIAN
% X_dot = [J]q_dot


syms q1 q2 q3 q4 q5 q6 q7 D1 D3 D5 D7

q17 = [q1; q2; q3; q4; q5; q6; q7];

T01 = [ -sin(q1) -cos(q1) 0 0 ; cos(q1) -sin(q1) 0 0 ; 0 0 1 D1 ; 0 0 0 1];
        
    % Transform 1 to 2 about -y1 z2
    T12 = [-sin(q2) -cos(q2) 0 0 ; 0 0 -1 0 ; cos(q2) -sin(q2)  0 0 ; 0 0 0 1];
    T02 = T01*T12;
    
    % Transform 2 to 3 about y2 z3
    T23 = [cos(q3) -sin(q3) 0 0 ; 0 0 -1 -D3 ; sin(q3) cos(q3) 0 0 ; 0 0 0 1];
    T03 = T02*T23;
    
    % Transform 3 to 4 about -y3 z4
    T34 = [sin(q4) cos(q4) 0 0 ; 0 0 1 0 ; cos(q4) -sin(q4) 0 0 ; 0 0 0 1];
    T04 = T03*T34;
    %
    % Transform 4 to 5 about y4 z5
    T45 = [cos(q5) -sin(q5) 0 0; 0 0 1 D5 ; -sin(q5) -cos(q5) 0 0 ; 0 0 0 1 ];
    T05 = T04*T45;
    
    % Transform 5 to 6 about -y5 z6
    T56 = [-sin(q6) -cos(q6) 0 0; 0 0 1 0 ; cos(q6) -sin(q6) 0 0 ; 0 0 0 1 ];
    T06 = T05*T56;
    
    % Transform 6 to 7 about y6 z7
    T67 = [ cos(q7) -sin(q7) 0 0; 0 0 -1 -D7 ; sin(q7) cos(q7) 0 0 ; 0 0 0 1 ];
    T07 = T06*T67;

    %%
    % T07 =
%       [ cos(q7)*(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) - sin(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) + cos(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), - sin(q7)*(cos(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) - sin(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) - sin(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - cos(q7)*(sin(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) + cos(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) - sin(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3))), D5*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) + D7*(sin(q6)*(cos(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) + cos(q2)*sin(q1)*sin(q4)) + cos(q6)*(cos(q5)*(sin(q4)*(cos(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - cos(q2)*cos(q4)*sin(q1)) - sin(q5)*(cos(q1)*cos(q3) - sin(q1)*sin(q2)*sin(q3)))) - D3*cos(q2)*sin(q1)]
%       [ cos(q7)*(cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) - sin(q6)*(cos(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - sin(q7)*(sin(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), - sin(q7)*(cos(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) - sin(q6)*(cos(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) - cos(q7)*(sin(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) + cos(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) + cos(q6)*(cos(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3))), D5*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) + D7*(sin(q6)*(cos(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) - cos(q1)*cos(q2)*sin(q4)) + cos(q6)*(cos(q5)*(sin(q4)*(sin(q1)*sin(q3) - cos(q1)*cos(q3)*sin(q2)) + cos(q1)*cos(q2)*cos(q4)) - sin(q5)*(cos(q3)*sin(q1) + cos(q1)*sin(q2)*sin(q3)))) + D3*cos(q1)*cos(q2)]
%       [                                                                                                                                                                   - sin(q7)*(sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - cos(q2)*cos(q5)*sin(q3)) - cos(q7)*(sin(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + cos(q2)*sin(q3)*sin(q5)) + cos(q6)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))),                                                                                                                                                                       sin(q7)*(sin(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + cos(q2)*sin(q3)*sin(q5)) + cos(q6)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4))) - cos(q7)*(sin(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) - cos(q2)*cos(q5)*sin(q3)),                                                                                                     cos(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)),                                                                                                                                            D1 - D5*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)) + D3*sin(q2) + D7*(cos(q6)*(cos(q5)*(cos(q4)*sin(q2) + cos(q2)*cos(q3)*sin(q4)) + cos(q2)*sin(q3)*sin(q5)) - sin(q6)*(sin(q2)*sin(q4) - cos(q2)*cos(q3)*cos(q4)))]
%       [                                                                                                                                                                                                                                                                                                                                                                                                                     0,                                                                                                                                                                                                                                                                                                                                                                                                                       0,                                                                                                                                                                                                                                                 0,                                                                                                                                                                                                                                                                                                                                                                1]
%  
    
 