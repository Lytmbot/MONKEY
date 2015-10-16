% HOW TO BASE SWAP....


% base 01 start posiition {0}
T_base_01 = [1 , 0 , 0 , 5 ;
             0 , 1 , 0 , 0 ;
             0 , 0 , 1 ,  2 ;
             0 , 0 , 0 ,  1 ];
       

% base 02 start posiition {0}
T_base_02 = [1 , 0 , 0 , 2 ;
             0 , 1 , 0 , 3 ;
             0 , 0 , 1 , 0 ;
             0 , 0 , 0 , 1 ];
         
T_base_02_2nd = [0 , 1 ,  0 , 0 ;
                 0 , 0 , -1 , 2 ;
                -1 , 0 ,  0 , 4 ;
                 0 , 0 ,  0 , 1 ];

time_end = 5;
coeffs = get_coefficients_LINEAR(T_base_02(1:3,4) ,T_base_02_2nd(1:3,4) , time_end);
k = get_K(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_f = get_THETAf(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_step = (theta_f/time_end);

for time = 0 : .1 : time_end
        
    theta = theta_step*(time );
    
    T_base_02(1,4) = get_points_LINEAR(coeffs(1,:),time) ;
    T_base_02(2,4) = get_points_LINEAR(coeffs(2,:),time) ;
    T_base_02(3,4) = get_points_LINEAR(coeffs(3,:),time) ;
    T_base_02(1:3,1:3) = get_R_it(k,theta) ;
        
    base_02 = T_base_01\T_base_02;
    RENDER(T_base_01,base_02,'down');
    
end
         
T_base_02_2nd = [0 , 1 ,  0 , 0 ;
                 0 , 0 , -1 , 0 ;
                -1 , 0 ,  0 , 4 ;
                 0 , 0 ,  0 , 1 ];

time_end = 2.5;
coeffs = get_coefficients_LINEAR(T_base_02(1:3,4) ,T_base_02_2nd(1:3,4) , time_end);

for time = 0 : .1 : time_end        
    theta = theta_step*(time );    
    T_base_02(1,4) = get_points_LINEAR(coeffs(1,:),time) ;
    T_base_02(2,4) = get_points_LINEAR(coeffs(2,:),time) ;
    T_base_02(3,4) = get_points_LINEAR(coeffs(3,:),time) ;         
    base_02 = T_base_01\T_base_02;
    RENDER(T_base_01,base_02,'down');    
end         
    

% SWAP
temp_1 = T_base_01;
temp_2 = T_base_02;
T_base_01 = temp_2;
T_base_02 = temp_1;
T_base_01(1:3,1:3) = T_base_01(1:3,1:3) *(-1);
T_base_02(1:3,1:3) = T_base_02(1:3,1:3) *(-1);
%base_02 = T_base_01\T_base_02;

T_base_02_2nd = [ 0,  1,  0, 0 ;
                  0,  0,  -1, 5 ;
                  1,  0,  0, 7 ;
                  0,  0,  0, 1 ];

time_end = 5;
coeffs = get_coefficients_LINEAR(T_base_02(1:3,4) ,T_base_02_2nd(1:3,4) , time_end);
k = get_K(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_f = get_THETAf(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_step = (theta_f/time_end);

for time = 0 : .1 : time_end        
    theta = theta_step*(time );    
    T_base_02(1,4) = get_points_LINEAR(coeffs(1,:),time) ;
    T_base_02(2,4) = get_points_LINEAR(coeffs(2,:),time) ;
    T_base_02(3,4) = get_points_LINEAR(coeffs(3,:),time) ; 
    T_base_02(1:3,1:3) = get_R_it(k,theta) ;    
    base_02 = T_base_01\T_base_02;
    RENDER(T_base_01,base_02,'up');    
end    

T_base_02_2nd = [ -1,  0,  0, 0 ;
                  0, 1,  0, 8 ;
                  0,  0, -1, 8 ;
                  0,  0,  0, 1 ];

time_end = 5;
coeffs = get_coefficients_LINEAR(T_base_02(1:3,4) ,T_base_02_2nd(1:3,4) , time_end);
k = get_K(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_f = get_THETAf(T_base_02(1:3,1:3) ,T_base_02_2nd(1:3,1:3));
theta_step = (theta_f/time_end);

for time = 0 : .1 : time_end        
    theta = theta_step*(time );    
    T_base_02(1,4) = get_points_LINEAR(coeffs(1,:),time) ;
    T_base_02(2,4) = get_points_LINEAR(coeffs(2,:),time) ;
    T_base_02(3,4) = get_points_LINEAR(coeffs(3,:),time) ; 
    T_base_02(1:3,1:3) = get_R_it(k,theta) ;    
    base_02 = T_base_01\T_base_02;
    RENDER(T_base_01,base_02,'up');    
end    








