

%INVERS BY SIMULTATIOUSE EQUAITONS

syms c1 s1 c2 s2 c3 s3 c4 s4 c5 s5 c6 s6 c7 s7 L01 L23 L45 L67



    % Transform 0 to 1 about Z z1
    T01 = [ -s1 -c1 0 0 ; c1 -s1 0 0 ; 0 0 1 L01 ; 0 0 0 1];
    
    % Transform 1 to 2 about -y1 z2
    T12 = [-s2 -c2 0 0 ; 0 0 -1 0 ; c2 -s2  0 0 ; 0 0 0 1];
    T02 = T01*T12;
    
    % Transform 2 to 3 about y2 z3
    T23 = [c3 -s3 0 0 ; 0 0 -1 -L23 ; s3 c3 0 0 ; 0 0 0 1];
    T03 = T02*T23;
    
    % Transform 3 to 4 about -y3 z4
    T34 = [s4 c4 0 0 ; 0 0 1 0 ; c4 -s4 0 0 ; 0 0 0 1];
    T04 = T03*T34;
    %
    % Transform 4 to 5 about y4 z5
    T45 = [c5 -s5 0 0; 0 0 1 L45 ; -s5 -c5 0 0 ; 0 0 0 1 ];
    T05 = T04*T45;
    
    % Transform 5 to 6 about -y5 z6
    T56 = [-s6 -c6 0 0 ; 
            0    0 1 0 ; 
            c6 -s6 0 0 ; 
            0    0 0 1 ];
        
    T06 = T05*T56;
    
    % Transform 6 to 7 about y6 z7
    T67 = [ c7 -s7 0 0; 0 0 -1 -L67 ; s7 c7 0 0 ; 0 0 0 1 ];
    T07 = T06*T67;
    
    T27 = T23*T34*T45*T56*T67;
    T04 = T01*T12*T23*T34;
    T47 = T45*T56*T67;
    
    T47b = inv(T01)*inv(T12)*inv(T23)*inv(T34)*T07;
    T47c = T01\T12\T23\T34\T07;
    
    T30 = inv(T03);
    
    
    R0a = [ 1  0   0   ; 
            0  c1 -s1  ;
            0  s1  c1 ];
        
    Rab = [ c2 -s2  0 ;
            s2  c2  0 ;
            0   0   1 ];
    Rb1 = [ c2  0  s2  ; 
            0   1  0   ;
           -s2  0  c2 ];
    
       R01 = R0a*Rab*Rb1;
       
       
       
       
       
       
       
       
       
       
       
    