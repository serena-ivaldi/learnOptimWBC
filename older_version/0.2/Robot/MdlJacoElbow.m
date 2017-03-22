%MDL_LBR4p Create model of LBR4+ manipulator
%
%      MdlJaco
%
% Script creates the variable LBR4p which describes the 
% kinematic and dynamic characteristics of a Kuka LBR4+ manipulator
% using standard DH conventions.
% The model  does NOT includes armature inertia and gear ratios.
%
% Also define the workspace vectors:
%   qz         starting configuration
%   qr         vertical strech configuration
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com



function [Jaco] = MdlJacoElbow()
   clear L
   deg = pi/180;
  
   % the x axes between matlab and v-rep are rotated of 180 degrees so for
   % this reason i have to invert al the d-h parameter to have the same
   % axis orientation.
   % to solve every issue is sufficient to do the d-h artenberg assignation
   % as doing before and then rotate the robot on v-rep of 180 degrees
   
   
   D1 = 0.2755;
   D2 = 0.4100;
   D3 = 0.2073;
   D4 = 0.0743;
   D5 = 0.0743;
   D6 = 0.1687;
   e2 = 0.0098;
   
   aa = (11.0 * pi)/ 72.0;
   ca = cos(aa);
   sa = sin(aa);
   c2a= cos(2*aa);
   s2a= sin(2*aa);
   d4b= D3 + (sa/s2a)*D4;
   d5b= (sa/s2a)*D4 + (sa/s2a)*D5;
   d6b= (sa/s2a)*D5 + D6;
   
   
   L(1) = Revolute('d', D1 , 'a', 0, 'alpha', pi/2,'flip', ...
       'qlim', [-180 180]*deg,...
       'I', [0.04,0.04,0.04], ...
       'r', [0 0 0], ...
       'm', 0.181);%,...   
%        'B', 1.48e-3, ...
%        'Tc', [0.395 -0.435],...
%        'Jm', 200e-6, ...
%        'G', -62.6111);

   L(2) = Revolute('d', 0, 'a', D2, 'alpha', +pi,'offset',-pi/2, ...
      'qlim', [47 313]*deg,...
       'I', [0.04,0.04,0.04], ...
       'r', [0 0 0], ...
       'm', 0.424);%,...
%        'B', .817e-3, ...
%        'Tc', [0.126 -0.071], ...
%        'Jm', 200e-6, ...
%        'G', 107.815);
       
   L(3) = Revolute('d',-e2, 'a',0, 'alpha', pi/2,'offset',+pi/2,  ...
      'qlim', [19 341]*deg,...
       'I', [0.04,0.04,0.04], ...
       'r', [0 0 0], ...
       'm', 0.211);%,...
%        'B', 1.38e-3, ...
%        'Tc', [0.132, -0.105],...
%        'Jm', 200e-6, ...
%        'G', -53.7063);
 L(4) = Revolute('d', -d4b, 'a', 0, 'alpha',2*aa,  ...
       'qlim', [-180 180]*deg,...
       'I', [0.04,0.04,0.04], ...
       'r', [0 0 0], ...
       'm', 0.069);%,...
%        'B', 71.2e-6, ...
%        'Tc', [11.2e-3, -16.9e-3],...
%        'Jm', 33e-6, ...
%        'G', 76.0364);
  

   %build the dynamical model of the robot
   Jaco = SerialLink(L, 'name', 'JacoElbow', ...
    'manufacturer', 'Kinova');
   %LBR4pModel.model3d = 'UNIMATE/puma560';
   
  
   % some useful poses
   %
   assignin('base', 'qr', [270 180 180 0 0 0]*deg);% extended arm
   assignin('base', 'qz', [30, 90, 90, 90, 0, 0]*deg); % start position
   assignin('base', 'qk', [180, 270, 90, 180, 0, 350]*deg); % start position
  
   

   clear L
   
end   
