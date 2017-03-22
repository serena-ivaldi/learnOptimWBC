function fkine = fkine(rob,q,tag)
%% FKINE - Forward kinematics solution including tool transformation for the LBR4p copy arm. 
% ========================================================================= 
%    
%    T = fkine(rob,q) 
%    T = rob.fkine(q) 
%    
%  Description:: 
%    Given a full set of joint variables the function 
%    computes the pose belonging to that joint with respect to the base frame. 
%    
%  Input:: 
%    rob: robot object of LBR4p copy specific class 
%    q:  7-element vector of generalized 
%         coordinates 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    T:  [4x4] Homogenous transformation matrix relating the pose of the tool 
%              for the given joint values to the base frame. 
%    
fkine = wholeBodyModel('forward-kinematics',reshape(rob.R_b,[],1),rob.x_b,q,tag);
% Obtaining the rotation matrix from root link to world frame
[x,R]    = frame2posrot(fkine);
fkine = eye(4);
fkine(1:3,1:3) = R;
fkine(1:3,4) = x;
end