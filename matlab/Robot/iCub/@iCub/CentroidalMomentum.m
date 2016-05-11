function [ Omega ] = CentroidalMomentum( rob,q,qd )
%WMB_CENTROIDALMOMENTUM computes the centroidal momentum of the system -
%functions of the state q, state derivative qDot, and floating base
%velocity vxb
%   Arguments : 
%       Optimised Mode : No arguments
%       Normal Mode : R - rotation from rootLink to world frame (3 x 3)
%                     p - translation from rootLink to world frame (3 x 1)
%                     qj - joint angles (NumDoF x 1)
%                     dqj - joint velocities (NumDoF x 1)
%                     vxb - floating base velocity (6 x 1)
%   Returns :   H - centroidal momentum (6 x 1)
%
% Author : Naveen Kuppuswamy (naveen.kuppuswamy@iit.it)
% Genova, Dec 2014

    
            Omega = wholeBodyModel('centroidal-momentum',reshape(rob.R_b,[],1),rob.x_b,q,qd,[rob.dx_b;rob.omega_W]);
end