function [] = RENDER()
% VERSION:
%   4
%   I N:
%       BASE_01 in frame {0} 
%       BASE_02 in frame {BASE_1} 


%   O U T:
%       nothing, just render yo bot
clear all;
%% ************************  I N I T I A L I S E  *************************
on = 1;
off = 0;

xy = [1,2,3]; % draw x y plane
xz = [1,3,2];
yz = [2,3,1];

%P L O T 3D
x_max = 21;
x_min = -21;
y_max = 21;
y_min = -21;
z_max = 21;
z_min = -1;


% will save every frame of motion for i in figure(1) as a img_counter.jpg
save_images = off;
img_counter = 1;

% I N V E R S E
show_P2 = off;      % point 2
show_P3 = off;      % point 3
show_E_set = on;    % point 4
show_a = on;        % angles
show_E_frame = off; % not finished
show_BASE = on;     % show the current base coords

% F O R W A R D
show_P3F = off;
show_E = on;

% O F F S E T S
q_offset = zeros(7,1);
q_offset(1) = -90;
q_offset(4) = -90;
q_offset(5) = -90;


d1 = 2;
d2 = 5;
d3 = 5;
d4 = 2;

%draw the points captured in the E path
draw = on;

% capture points touched by E
capture_x = [1];
capture_y = [1];
capture_z = [1];
capture_counter = 1;

just_rotation = [1:3,1:3];

%% SET UP
    
% frame {0} origin to BASE 01 Tog0
BASE_01 = [1 0 0 -2 ;
           0 1 0 -2 ;
           0 0 1  8 ;
           0 0 0  1 ];
      
% BASE 01 to endofector in frame {BASE_01}    T07  
BASE_02 = [1 0 0  5 ;
           0 1 0  5 ;
           0 0 1 -2 ;
           0 0 0  1 ];

T_0_BASE02 = BASE_01*BASE_02;


Ex = T_0_BASE02(1,4);
Ey = T_0_BASE02(2,4);
Ez = T_0_BASE02(3,4);


%% INVERSE KINEMATICS
% now can start in frame {BASE_01} and get all joint q1-7 locations.

qs = I_KINEMATICS_getangles(BASE_02);
T = F_KINEMATICS_get_transfers(qs);
        
%% ****************************   G E T  P 3   *****************************
         
        % vector from E to P3 in frame {7} the wrist
        % projetion back along z7 to get P3 in frame {7}
        rEP3_7 = [0; 0; -d4; 1] ;
        
        % then translate into frame {0} to get vector from 0 to P3 in {0}
        r0P3 = T{7}*rEP3_7;
        P3 = [r0P3(1) ; r0P3(2) ; r0P3(3) ; 1];
        
%% ****************************  G E T   P 1  *****************************

        P1 = [T{1}(1,4) ; T{1}(2,4) ; T{1}(3,4); 1];
      
%% ****************************  G E T   P 2  *****************************
       
        % get P2 in {1}
        P2 = [ T{3}(1,4); T{3}(2,4); T{3}(3,4); 1];
        
%% ************  O B T A I N    J O I N T    L O C A T I O N S  ***********
        
    T_0 = cell(7,1);
    
    for i = 1:1:7
        T_0{i} = BASE_01*T{i};
    end

        % origin
        x0 = BASE_01(1,4);
        y0 = BASE_01(2,4);
        z0 = BASE_01(3,4);
        
        % joint 01 locaiton
        x1 = T_0{1}(1,4);
        y1 = T_0{1}(2,4);
        z1 = T_0{1}(3,4);
        
        % joint 02 locaiton
        x2 = T_0{3}(1,4);
        y2 = T_0{3}(2,4);
        z2 = T_0{3}(3,4);
        
        % joint 03 locaiton
        x3 = T_0{5}(1,4);
        y3 = T_0{5}(2,4);
        z3 = T_0{5}(3,4);
        
        % endofactor locaiton (gripper 01)
        x4 = T_0{7}(1,4);
        y4 = T_0{7}(2,4);
        z4 = T_0{7}(3,4);
        
        %Forward link possition from invers angles
        x_pos = [x0,x1,x2,x3,x4];
        y_pos = [y0,y1,y2,y3,y4];
        z_pos = [z0,z1,z2,z3,z4];
       
%% ********************  R E N D E R   G R I P P E R S  *******************
        % Gripper 02
        RG = [ 0; -1; 1; 1];
        TG = T{7}*RG;
        GLx3 = TG(1);
        GLy3 = TG(2);
        GLz3 = TG(3);
        
        RG = [0; -1; 0 ; 1];
        TG = T{7}*RG;
        GLx0 = TG(1);
        GLy0 = TG(2);
        GLz0 = TG(3);
        
        RG = [0; 1; 0 ; 1];
        TG = T{7}*RG;
        GLx1 = TG(1);
        GLy1 = TG(2);
        GLz1 = TG(3);
        
        RG = [0; 1; 1; 1];
        TG = T{7}*RG;
        GLx2 = TG(1);
        GLy2 = TG(2);
        GLz2 = TG(3);
        
        G2x_pos = [GLx3 GLx0 GLx1 GLx2];
        G2y_pos = [GLy3 GLy0 GLy1 GLy2];
        G2z_pos = [GLz3 GLz0 GLz1 GLz2];
        
        % Gripper 01
        G1x_pos = [0 0 0 0];
        G1y_pos = [1 1 -1 -1];
        G1z_pos = [-1 0 0 -1];
        
        
%% **********************  D R A W    T H E    B O T  *********************
        
        hold off
        pause(0.001) % delay render so its clear
        figure(1);
       
        %the robot
        plot3(x_pos,y_pos,z_pos, G2x_pos,G2y_pos,G2z_pos, G1x_pos,G1y_pos,G1z_pos,capture_x,capture_y,capture_z);
        hold on; 
        
        %the frame and building
        %corner locations
        bar = 10;
        jacked_up = 2;
        color = [0,0,0];
           
        FRAME = get_FRAME(bar,jacked_up);                
        BOT = FRAME{1}; 
        TOP = FRAME{2};
        LEG = FRAME{3};
        plot3(BOT(:,1),BOT(:,2),BOT(:,3) , TOP(:,1),TOP(:,2),TOP(:,3),  'Color', color);
        for i = 1:1:6
            plot3( [BOT(i,1),TOP(i,1),LEG(i,1)] , [BOT(i,2),TOP(i,2),LEG(i,2)] , [BOT(i,3),TOP(i,3),LEG(i,3)] ,'Color',color );
        
        end
        
        axis([x_min,x_max,y_min,y_max,z_min,z_max])
        xlabel('X','fontsize',16,'fontweight','bold','color',[1 .1 1])
        ylabel('Y','fontsize',16,'fontweight','bold','color',[1 .1 1])
        zlabel('Z','fontsize',16,'fontweight','bold','color',[1 .1 1])
        
        
        % show desired Endofactor location
        if show_E_set == on
            label_pos_P3 = ['POINT TO TRACK:', sprintf('\nEx= '), num2str(round(Ex,2)), sprintf('\nEy= '), num2str(round(Ey,2)), sprintf('\nEz= '), num2str(round(Ez,2)), ''];
            text(x_min+1,y_max-1,z_max/1.2, label_pos_P3)
        end
        
        % show Base possition FORWARD
        if show_BASE == on
            label_pos_E = ['    Bx= ', num2str(round(x0,2)), sprintf('\n    By= '), num2str(round(y0,2)), sprintf('\n    Bz= '), num2str(round(z0,2)), ''];
            text(x0,y0,z0, label_pos_E)
        end
        
        % show Endefactor possition FORWARD
        if show_E == on
            label_pos_E = ['    FEx= ', num2str(round(x4,2)), sprintf('\n    FEy= '), num2str(round(y4,2)), sprintf('\n    FEz= '), num2str(round(z4,2)), ''];
            text(x4,y4,z4, label_pos_E)
        end
        
        % show P3 from FORWARD
        if show_P3F == on
            label_pos_P3F = ['    Fx3= ', num2str(round(x3,2)), sprintf('\n    Fy3= '), num2str(round(y3,2)), sprintf('\n    Fz3= '), num2str(round(z3,2)), ''];
            text(x3,y3,z3, label_pos_P3F)
        end
        
        % show point 2 INVERSE
        if show_P2 == on
            label_pos_P2 = ['    Ix2= ', num2str(round(P2(1),2)), sprintf('\n    Iy2= '), num2str(round(P2(2),2)), sprintf('\n    Iz2= '), num2str(round(P2(3),2)), ''];
            text(P2(1),P2(2),P2(3), label_pos_P2)
        end
        
        % show point 3 from INVERSE
        if show_P3 == on
            label_pos_P3 = ['    Ix3= ', num2str(round(P3(1),2)), sprintf('\n    Iy3= '), num2str(round(P3(2),2)), sprintf('\n    Iz3= '), num2str(round(P3(3),2)), ''];
            text(P3(1),P3(2),P3(3), label_pos_P3)
        end
               
        % show q1-7 values top left corner from INVERSE
        if show_a == on
            label_angles = [sprintf('ANGLES:\nq1 = '), num2str(round(qs(1),2)), sprintf('\nq2 = '), num2str(round(qs(2),2)), sprintf('\nq3 = '), num2str(round(qs(3),2)),sprintf('\nq4 = '), num2str(round(qs(4),2)), sprintf('\nq5 = '), num2str(round(qs(5),2)), sprintf('\nq6 = '), num2str(round(qs(6),2)),sprintf('\nq7 = '), num2str(round(qs(7),2))];
            text(x_min+1,y_max-1,z_max/3,label_angles);
        end
        
        %save figure(1) of robot motion for gif creation
        if save_images == on
            lable_for_plot = [num2str(img_counter),'.jpg'];
            saveas(1, lable_for_plot);
            img_counter = img_counter + 1;
        end
        
        
       





