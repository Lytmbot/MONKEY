function [ G_pos ] = get_GRIPPER( ID, BASE )
%I N:
%   Gripper ID (1 or 2) to render and T {0} to {BASE} (4x4) 
%
% O U T:
%   G_pos in frame {0}

% Gripper   
G_pos = [ 0  0  0  0  ; 
         -1 -1  1  1  ; 
         ID  0  0  ID ;
          1  1  1  1 ];
G_pos(:,1) = BASE*G_pos(:,1);                       
G_pos(:,2) = BASE*G_pos(:,2);        
G_pos(:,3) = BASE*G_pos(:,3);        
G_pos(:,4) = BASE*G_pos(:,4);
end

