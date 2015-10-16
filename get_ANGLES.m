%VERSION:
%   4
%   forward_n_back
%   push angles into forward kinematics and then use resulting point and
%   T07 value to calulate aangles with inverse kinematics
clear all;

%% ************************  I N I T I A L I S E  *************************
on = 1;
off = 0;

%P L O T 3D
x_max = 15;
x_min = -15;
y_max = 15;
y_min = -15;
z_max = 15;
z_min = -1;


% will save every frame of motion for i in figure(1) as a img_counter.jpg
save_images = off;
img_counter = 0;

% I N V E R S E
show_P2 = off;      % point 2
show_P3 = off;      % point 3
show_E_set = on;    % point 4
show_a = on;        % angles
show_E_frame = off; % not finished

% F O R W A R D
show_P3F = off;
show_E = on;

% O F F S E T S
q_offset = zeros(7,1);
q_offset(1) = -90;
q_offset(4) = -90;
q_offset(5) = -90;
q_offset(6) = 0;

% origin
x0 = 0;
y0 = 0;
z0 = 0;

%link lengths
D1 = 2;
D3 = 5;
D5 = 5;
D7 = 2;

T07_set = zeros(4,4);

% capture points touched by E
capture_x = [1];
capture_y = [1];
capture_z = [1];
counter = 1;

%%
for i = -180 : 2 : 180
%% **************************  S E T   E(x,y,z)  **************************

    q = [-45,90,0,-i*2,45,i,0];
    
    T07_desired = F_KINEMATICS_getpoints(q);
    
    % vector from 0 to desired endofactor position
    Ex = T07_desired(1,4);
    Ey = T07_desired(2,4);
    Ez = T07_desired(3,4);
    r0E = [Ex ; Ey ; Ez; 1];
    T07_set = T07_desired;
   
%% ****************************   G E T  P 3   ****************************
    
    % vector from E to P3 in frame {7} the wrist
    % projetion back along z7 to get P3 in frame {7}
    rEP3_7 = [0; 0; -D7; 1] ;
    
    % then translate into frame {0} to get vector from 0 to P3 in {0}
    r0P3 = T07_set*rEP3_7;
    P3 = [r0P3(1) ; r0P3(2) ; r0P3(3) ; 1];
    
%% ****************************  G E T   q 1  *****************************
    
    % rotate q1 s.t. x1 aligns with P3, in {0}
    q1 = atan2d(P3(2), P3(1)) + q_offset(1);
    
    % Transform {0} to {1} about Z z1
    T01 = [ -sind(q1) -cosd(q1) 0 0 ; cosd(q1) -sind(q1) 0 0 ; 0 0 1 D1 ; 0 0 0 1];
    
    % point 01 locaiton in {0}
    P1 = [T01(1,4) ; T01(2,4) ; T01(3,4); 1];
    
%% ****************************  G E T   q 2  *****************************
    
    %Move everything to frame {1}
    P1_1 = T01\P1;
    P3_1 = T01\P3;
    
    %distance P1 to P3
    h = ((P3_1(1))^2 + (P3_1(3))^2)^0.5;
    theta_01 = atan2d(P3_1(3) , P3_1(1));
    theta_02 = acosd((h/2)/D3);
   % theta_02 = atan2d( ((L23^2 - (h/2)^2)^0.5 ) , h/2 );
    
    % elbow up
    q2 = theta_01 + theta_02;
    
%% ****************************  G E T   q 3  *****************************
    % set a3.... becaues I dont know what else to do
    q3 = 0;
    
%% ****************************  G E T   q 4  *****************************
    % Transform 0 to 3
    T12 = [-sind(q2) -cosd(q2) 0 0 ; 0 0 -1 0 ; cosd(q2) -sind(q2)  0 0 ; 0 0 0 1];
    T23 = [cosd(q3) -sind(q3) 0 0 ; 0 0 -1 -D3 ; sind(q3) cosd(q3) 0 0 ; 0 0 0 1];
    T03 = T01*T12*T23;
    
    % get P2 in {1}
    P2 = [ T03(1,4); T03(2,4); T03(3,4); 1];
    P2_1 = T01\P2;
    
    q4 = q_offset(4) - 2*theta_02;
    
    T34 = [sind(q4) cosd(q4) 0 0 ; 0 0 1 0 ; cosd(q4) -sind(q4) 0 0 ; 0 0 0 1];
    T04 = T03*T34;
    
%% ************************  G E T   q 5 6 and 7  *************************
    
    r0E_4 = T04\r0E;
    q5 = q_offset(5)+atan2d(r0E_4(1), r0E_4(3));
    T45 = [cosd(q5) -sind(q5) 0 0; 0 0 1 D5 ; -sind(q5) -cosd(q5) 0 0 ; 0 0 0 1 ];
    
    % need T47
    T47 = T04\T07_set;
          
%   T57 =   
%   [ -c7*s6,  s6*s7, c6, L67*c6]
%   [     s7,     c7,  0,      0]
%   [  c6*c7, -c6*s7, s6, L67*s6]
%   [      0,      0,  0,      1]
%     
    T05 = T04*T45;
    T57 = T05\T07_set;
    q6 = atan2d(T57(3,3), T57(1,3));
    
    T56 = [-sind(q6) -cosd(q6) 0 0; 0 0 1 0 ; cosd(q6) -sind(q6) 0 0 ; 0 0 0 1 ];
    T67 = (T45*T56)\T47;
    % T67 =
    % [ c7, -s7,  0,    0]
    % [  0,   0, -1, -L67]
    % [ s7,  c7,  0,    0]
    % [  0,   0,  0,    1]
       
    q7 = atan2d(T67(3,1),T67(1,1));
        
%% ******************  T R A N S F O R M     M A T R I X  ******************
    
    % Transform 0 to 1 about Z z1
    T01 = [ -sind(q1) -cosd(q1) 0 0 ; cosd(q1) -sind(q1) 0 0 ; 0 0 1 D1 ; 0 0 0 1];
    
    % Transform 1 to 2 about -y1 z2
    T12 = [-sind(q2) -cosd(q2) 0 0 ; 0 0 -1 0 ; cosd(q2) -sind(q2)  0 0 ; 0 0 0 1];
    T02 = T01*T12;
    
    % Transform 2 to 3 about y2 z3
    T23 = [cosd(q3) -sind(q3) 0 0 ; 0 0 -1 -D3 ; sind(q3) cosd(q3) 0 0 ; 0 0 0 1];
    T03 = T02*T23;
    
    % Transform 3 to 4 about -y3 z4
    T34 = [sind(q4) cosd(q4) 0 0 ; 0 0 1 0 ; cosd(q4) -sind(q4) 0 0 ; 0 0 0 1];
    T04 = T03*T34;
    %
    % Transform 4 to 5 about y4 z5
    T45 = [cosd(q5) -sind(q5) 0 0; 0 0 1 D5 ; -sind(q5) -cosd(q5) 0 0 ; 0 0 0 1 ];
    T05 = T04*T45;
    
    % Transform 5 to 6 about -y5 z6
    T56 = [-sind(q6) -cosd(q6) 0 0; 0 0 1 0 ; cosd(q6) -sind(q6) 0 0 ; 0 0 0 1 ];
    T06 = T05*T56;
    
    % Transform 6 to 7 about y6 z7
    T67 = [ cosd(q7) -sind(q7) 0 0; 0 0 -1 -D7 ; sind(q7) cosd(q7) 0 0 ; 0 0 0 1 ];
    T07 = T06*T67;
    
%% ************  O B T A I N    J O I N T    L O C A T I O N S  ************
    
    % origin
    x0 = 0;
    y0 = 0;
    z0 = 0;
    
    % joint 01 locaiton
    x1 = T01(1,4);
    y1 = T01(2,4);
    z1 = T01(3,4);
    
    % joint 02 locaiton
    x2 = T03(1,4);
    y2 = T03(2,4);
    z2 = T03(3,4);
    
    % joint 03 locaiton
    x3 = T05(1,4);
    y3 = T05(2,4);
    z3 = T05(3,4);
    
    % endofactor locaiton (gripper 01)
    x4 = T07(1,4);
    y4 = T07(2,4);
    z4 = T07(3,4);
    
    %Forward link possition from invers angles
    x_pos = [x0,x1,x2,x3,x4];
    y_pos = [y0,y1,y2,y3,y4];
    z_pos = [z0,z1,z2,z3,z4];
    
%% ********************  R E N D E R   G R I P P E R S  ********************
    % Gripper 02
    RG = [1 0 0 0; 0 1 0 -1; 0 0 1 1 ; 0 0 0 1];
    TG = T07*RG;
    GLx3 = TG(1,4);
    GLy3 = TG(2,4);
    GLz3 = TG(3,4);
    
    RG = [1 0 0 0; 0 1 0 -1; 0 0 1 0 ; 0 0 0 1];
    TG = T07*RG;
    GLx0 = TG(1,4);
    GLy0 = TG(2,4);
    GLz0 = TG(3,4);
    
    RG = [1 0 0 0; 0 1 0 1; 0 0 1 0 ; 0 0 0 1];
    TG = T07*RG;
    GLx1 = TG(1,4);
    GLy1 = TG(2,4);
    GLz1 = TG(3,4);
    
    RG = [1 0 0 0; 0 1 0 1; 0 0 1 1 ; 0 0 0 1];
    TG = T07*RG;
    GLx2 = TG(1,4);
    GLy2 = TG(2,4);
    GLz2 = TG(3,4);
    
    G2x_pos = [GLx3 GLx0 GLx1 GLx2];
    G2y_pos = [GLy3 GLy0 GLy1 GLy2];
    G2z_pos = [GLz3 GLz0 GLz1 GLz2];
    
    % Gripper 01
    G1x_pos = [0 0 0 0];
    G1y_pos = [1 1 -1 -1];
    G1z_pos = [-1 0 0 -1];
    
    %Points 1 - E to check against
    r03x = [P1(1) , P2(1), P3(1), Ex];
    r03y = [P1(2) , P2(2), P3(2), Ey];
    r03z = [P1(3) , P2(3), P3(3), Ez];
    
%% **********************  D R A W    T H E    B O T  **********************
    pause(0.001) % delay render so its clear
    figure(1);
    plot3(r03x,r03y,r03z, x_pos,y_pos,z_pos, G2x_pos,G2y_pos,G2z_pos, G1x_pos,G1y_pos,G1z_pos,capture_x,capture_y,capture_z);
    axis([x_min,x_max,y_min,y_max,z_min,z_max])
    xlabel('X','fontsize',16,'fontweight','bold','color',[1 .1 1])
    ylabel('Y','fontsize',16,'fontweight','bold','color',[1 .1 1])
    zlabel('Z','fontsize',16,'fontweight','bold','color',[1 .1 1])
    
    
    % show desired Endofactor location
    if show_E_set == on
        label_pos_P3 = ['POINT TO TRACK:', sprintf('\nEx= '), num2str(round(Ex,2)), sprintf('\nEy= '), num2str(round(Ey,2)), sprintf('\nEz= '), num2str(round(Ez,2)), ''];
        text(x_min+1,y_max-1,z_max-2, label_pos_P3)
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
    
    % show E frame rotation matrix relative to {0} NOT SET YET
    if show_E_frame == on
        label_angles = [sprintf('ANGLES:\nq1 = '), num2str(round(q1,2)), sprintf('\nq2 = '), num2str(round(q2,2)), sprintf('\nq3 = '), num2str(round(q3,2)),sprintf('\nq4 = '), num2str(round(q4,2)), sprintf('\nq5 = '), num2str(round(q5,2)), sprintf('\nq6 = '), num2str(round(q6,2)),sprintf('\nq7 = '), num2str(round(q7,2))];
        text(-19,19,8,label_angles);
    end
    
    % show q1-7 values top left corner from INVERSE
    if show_a == on
        label_angles = [sprintf('ANGLES:\nq1 = '), num2str(round(q1,2)), sprintf('\nq2 = '), num2str(round(q2,2)), sprintf('\nq3 = '), num2str(round(q3,2)),sprintf('\nq4 = '), num2str(round(q4,2)), sprintf('\nq5 = '), num2str(round(q5,2)), sprintf('\nq6 = '), num2str(round(q6,2)),sprintf('\nq7 = '), num2str(round(q7,2))];
        text(x_min+1,y_max-1,z_max-9,label_angles);
    end
    
    %save figure(1) of robot motion for gif creation
    if save_images == on
        lable_for_plot = [num2str(img_counter),'.jpg'];
        saveas(1, lable_for_plot);
        img_counter = img_counter + 1;
    end
    
    if T07_set ~= T07_desired;
        disp('mismatch');
    end
    
    capture_x(counter) = T07(1,4);
    capture_y(counter) = T07(2,4);   
    capture_z(counter) = T07(3,4);
    counter =counter + 1;
    
end
%% E N D    I N V E R S E
%% JUNK



