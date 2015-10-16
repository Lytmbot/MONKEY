function [T] = F_KINEMATICS_get_transfers(q)
%%********************  W H A T   D O E S   I T   D O  ********************

% I N:
% q1 -> q7 

% O U T:
% returns a transfer matrix T07_desired that captures rotation between 
% frame {0} and frame {E} and endofactor location E(x,y,z)
% T{7,1} where T{n} is the transfer matrix from 0 -> n
%in the form:
%       | Rxx_n , Ryx_n , Rzx_n , Ex_n |
%       | Rxy_n , Ryy_n , Rzy_n , Ey_n |
%       | Rxz_n , Ryz_n , Rzz_n , EZ_n |
%       |     0 ,     0 ,     0 , 1    |

%% ************************  L I N K   L E N G T H  ************************ 
    L01 = 2;       
    L23 = 5;
    L45 = 5;
    L67 = 2;
        
%% ******************  T R A N S F O R M     M A T R I X  ******************   

    T = cell(7,1);

    % Transform 0 to 1 about Z z1
    T{1} = [ -sind(q(1)) -cosd(q(1)) 0 0 ; cosd(q(1)) -sind(q(1)) 0 0 ; 0 0 1 L01 ; 0 0 0 1];
        
    % Transform 1 to 2 about -y1 z2
    T12 = [-sind(q(2)) -cosd(q(2)) 0 0 ; 0 0 -1 0 ; cosd(q(2)) -sind(q(2))  0 0 ; 0 0 0 1];
    T{2} = T{1}*T12;
    
    % Transform 2 to 3 about y2 z3
    T23 = [cosd(q(3)) -sind(q(3)) 0 0 ; 0 0 -1 -L23 ; sind(q(3)) cosd(q(3)) 0 0 ; 0 0 0 1];
    T{3} = T{2}*T23;
    
    % Transform 3 to 4 about -y3 z4
    T34 = [sind(q(4)) cosd(q(4)) 0 0 ; 0 0 1 0 ; cosd(q(4)) -sind(q(4)) 0 0 ; 0 0 0 1];
    T{4} = T{3}*T34;
    %
    % Transform 4 to 5 about y4 z5
    T45 = [cosd(q(5)) -sind(q(5)) 0 0; 0 0 1 L45 ; -sind(q(5)) -cosd(q(5)) 0 0 ; 0 0 0 1 ];
    T{5} = T{4}*T45;
    
    % Transform 5 to 6 about -y5 z6
    T56 = [-sind(q(6)) -cosd(q(6)) 0 0; 0 0 1 0 ; cosd(q(6)) -sind(q(6)) 0 0 ; 0 0 0 1 ];
    T{6} = T{5}*T56;
    
    % Transform 6 to 7 about y6 z7
    T67 = [ cosd(q(7)) -sind(q(7)) 0 0; 0 0 -1 -L67 ; sind(q(7)) cosd(q(7)) 0 0 ; 0 0 0 1 ];
    T{7} = T{6}*T67;
        
end
