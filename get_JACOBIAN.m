function [ Jacobian ] = get_JACOBIAN( T07)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


D = [2,5,5,2];
w = [0;0;1];

Jacobian = zeros(6,7);
T = cell(7,1);
T_0to = cell(7,1);

% gives me the angels
qs = I_KINEMATICS_getangles(T07);

% Transform 0 to 1 about Z z1
T{1} = [ -sind(qs(1)) -cosd(qs(1)) 0 0 ; cosd(qs(1)) -sind(qs(1)) 0 0 ; 0 0 1 D(1) ; 0 0 0 1];

% Transform 1 to 2 about -y1 z2
T{2} = [-sind(qs(2)) -cosd(qs(2)) 0 0 ; 0 0 -1 0 ; cosd(qs(2)) -sind(qs(2))  0 0 ; 0 0 0 1];

% Transform 2 to 3 about y2 z3
T{3} = [cosd(qs(3)) -sind(qs(3)) 0 0 ; 0 0 -1 -D(2) ; sind(qs(3)) cosd(qs(3)) 0 0 ; 0 0 0 1];

% Transform 3 to 4 about -y3 z4
T{4} = [sind(qs(4)) cosd(qs(4)) 0 0 ; 0 0 1 0 ; cosd(qs(4)) -sind(qs(4)) 0 0 ; 0 0 0 1];

% Transform 4 to 5 about y4 z5
T{5} = [cosd(qs(5)) -sind(qs(5)) 0 0; 0 0 1 D(3) ; -sind(qs(5)) -cosd(qs(5)) 0 0 ; 0 0 0 1 ];

% Transform 5 to 6 about -y5 z6
T{6} = [-sind(qs(6)) -cosd(qs(6)) 0 0; 0 0 1 0 ; cosd(qs(6)) -sind(qs(6)) 0 0 ; 0 0 0 1 ];

% Transform 6 to 7 about y6 z7
T{7} = [ cosd(qs(7)) -sind(qs(7)) 0 0; 0 0 -1 -D(4) ; sind(qs(7)) cosd(qs(7)) 0 0 ; 0 0 0 1 ];


T_0i = eye(4);

for current_col = 1:1:7
    
    % T_0i = T_0(i-1)*T_(i-1)7
    T_0i = T_0i*T{current_col};

    % T_i7 = T_(i-1)i*T_(i-1)7
    T_i7 = T_0i\T07;

    % get E in frame {i}    
    r_iE = T_i7(1:3,4);         
    v_iE = cross(w,r_iE); 
    %in frame {0}
    v_iE = T_0i(1:3,1:3)*v_iE;
    w_0i = T_0i(1:3,1:3)*w;
            
    for i = 1 : 1 : 3
        Jacobian(i,current_col) = v_iE(i);
        Jacobian(i+3,current_col) = w_0i(i);
    end
    
end



