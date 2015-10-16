function [ FRAME ] = get_FRAME(bar,offset )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

        jacked_up = zeros(7,3);       
        jacked_up(:,3) = offset;
                   
        FRAME = cell(2,1);
                
        FRAME{1} = [0 0 0; 0 bar 0 ; 0 bar*2 0 ;  bar bar*2 0; bar bar 0; bar 0 0; 0 0 0] + jacked_up ;
        FRAME{2} = [0 0 bar; 0 bar bar ; 0 bar*2 bar ; bar bar*2 bar; bar bar bar; bar 0 bar ; 0 0 bar] + jacked_up;
        FRAME{3} = [0 0 0; 0 bar 0 ; 0 bar*2 0 ;  bar bar*2 0; bar bar 0; bar 0 0; 0 0 0];
         
end

