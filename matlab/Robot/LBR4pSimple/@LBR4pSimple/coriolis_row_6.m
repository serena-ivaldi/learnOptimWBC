function Crow = coriolis_row_6(rob,in2,in3)
%% CORIOLIS_ROW_6 - Computation of the robot specific Coriolis matrix row for joint 6 of 7. 
% ========================================================================= 
%    
%    Crow = coriolis_row_6(rob,q,qd) 
%    Crow = rob.coriolis_row_6(q,qd) 
%    
%  Description:: 
%    Given a full set of joint variables and their first order temporal derivatives this function computes the 
%    Coriolis matrix row number 6 of 7 for LBR4pSimple copy. 
%    
%  Input:: 
%    rob: robot object of LBR4pSimple copy specific class 
%    qd:  7-element vector of generalized 
%    q:  7-element vector of generalized 
%    
%  Output:: 
%    Crow:  [1x7] row of the robot Coriolis matrix 
%    
%  Example:: 
%    --- 
%    
%  Known Bugs:: 
%    --- 
%    
%  TODO:: 
%    --- 
%    
%  References:: 
%    1) Robot Modeling and Control - Spong, Hutchinson, Vidyasagar 
%    2) Modelling and Control of Robot Manipulators - Sciavicco, Siciliano 
%    3) Introduction to Robotics, Mechanics and Control - Craig 
%    4) Modeling, Identification & Control of Robots - Khalil & Dombre 
%    
%  Authors:: 
%    This is an autogenerated function. 
%    Code generator written by: 
%    Joern Malzahn 
%    2012 RST, Technische Universitaet Dortmund, Germany 
%    http://www.rst.e-technik.tu-dortmund.de 
%    
%  See also coriolis.
%    
    
% Copyright (C) 1993-2016, by Peter I. Corke 
% Copyright (C) 2012-2016, by Joern Malzahn 
% 
% This file has been automatically generated with The Robotics Toolbox for Matlab (RTB). 
% 
% RTB and code generated with RTB is free software: you can redistribute it and/or modify 
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
% 
% The code generation module emerged during the work on a project funded by 
% the German Research Foundation (DFG, BE1569/7-1). The authors gratefully  
% acknowledge the financial support. 

%% Bugfix
%  In some versions the symbolic toolbox writes the constant $pi$ in
%  capital letters. This way autogenerated functions might not work properly.
%  To fix this issue a local variable is introduced:
PI = pi;
   




%    This function was generated by the Symbolic Math Toolbox version 5.10.
%    25-Feb-2016 23:09:49

q2 = in2(:,2);
q3 = in2(:,3);
q4 = in2(:,4);
q5 = in2(:,5);
q6 = in2(:,6);
qd1 = in3(:,1);
qd2 = in3(:,2);
qd3 = in3(:,3);
qd4 = in3(:,4);
qd5 = in3(:,5);
qd7 = in3(:,7);
t2 = sin(q6);
t3 = cos(q3);
t4 = cos(q2);
t5 = t3.^2;
t6 = t4.^2;
t7 = cos(q4);
t8 = t7.^2;
t9 = cos(q5);
t10 = sin(q4);
t11 = sin(q2);
t12 = sin(q5);
t13 = sin(q3);
t14 = cos(q6);
t15 = t14.^2;
t16 = t9.^2;
t17 = q6.*2.0;
t18 = sin(t17);
Crow = [qd1.*t2.*9.126e-3+qd1.*t18.*9.126e-4-qd1.*t2.*t5.*9.126e-3+qd3.*t2.*t4.*9.126e-3+qd1.*t2.*t7.*9.36e-3+qd1.*t2.*t5.*t6.*9.126e-3+qd1.*t2.*t5.*t8.*9.126e-3-qd1.*t2.*t6.*t7.*9.36e-3-qd1.*t2.*t6.*t8.*9.126e-3-qd3.*t2.*t4.*t8.*9.126e-3+qd7.*t2.*t4.*t7.*(3.0./1.0e2)-qd1.*t2.*t5.*t14.*3.6504e-3-qd1.*t2.*t6.*t14.*1.8252e-3+qd3.*t3.*t9.*t11.*(1.9e1./1.0e2)+qd4.*t4.*t7.*t12.*(1.9e1./1.0e2)+qd2.*t4.*t9.*t13.*1.918252e-1+qd5.*t4.*t9.*t10.*1.918252e-1+qd4.*t2.*t11.*t13.*9.126e-3-qd1.*t2.*t14.*t16.*1.8252e-3+qd1.*t9.*t10.*t14.*9.36e-3-qd2.*t10.*t11.*t12.*1.918252e-1-qd5.*t11.*t12.*t13.*1.918252e-1-qd1.*t2.*t5.*t6.*t8.*9.126e-3+qd1.*t2.*t5.*t6.*t14.*3.6504e-3-qd1.*t3.*t4.*t9.*t11.*1.8252e-3+qd2.*t3.*t4.*t7.*t12.*(1.9e1./1.0e2)+qd1.*t2.*t5.*t8.*t14.*1.8252e-3-qd1.*t2.*t6.*t8.*t14.*1.8252e-3+qd2.*t2.*t3.*t11.*t13.*9.126e-3-qd2.*t2.*t4.*t10.*t13.*9.36e-3-qd3.*t2.*t4.*t8.*t14.*1.8252e-3-qd1.*t5.*t7.*t9.*t10.*1.8252e-3-qd5.*t2.*t4.*t7.*t14.*1.8252e-3+qd1.*t6.*t7.*t9.*t10.*1.8252e-3+qd3.*t4.*t7.*t9.*t10.*1.8252e-3-qd7.*t2.*t3.*t10.*t11.*(3.0./1.0e2)+qd3.*t3.*t8.*t9.*t11.*1.8252e-3+qd2.*t3.*t4.*t12.*t14.*9.36e-3+qd5.*t3.*t7.*t9.*t11.*1.918252e-1-qd2.*t4.*t8.*t9.*t13.*1.8252e-3+qd1.*t2.*t5.*t14.*t16.*1.8252e-3+qd1.*t2.*t6.*t14.*t16.*3.6504e-3+qd1.*t3.*t10.*t12.*t13.*1.8252e-3+qd3.*t2.*t4.*t14.*t16.*1.8252e-3-qd1.*t6.*t9.*t10.*t14.*9.36e-3+qd2.*t5.*t10.*t11.*t12.*1.8252e-3+qd3.*t3.*t9.*t11.*t14.*9.126e-3-qd4.*t3.*t10.*t11.*t12.*(1.9e1./1.0e2)+qd3.*t3.*t9.*t11.*t15.*1.8252e-3+qd4.*t4.*t7.*t12.*t14.*9.126e-3+qd4.*t4.*t7.*t12.*t15.*1.8252e-3-qd2.*t4.*t9.*t13.*t15.*1.8252e-3-qd5.*t4.*t9.*t10.*t15.*1.8252e-3+qd4.*t2.*t11.*t13.*t14.*1.8252e-3+qd7.*t4.*t9.*t10.*t14.*(3.0./1.0e2)-qd3.*t7.*t11.*t12.*t13.*1.918252e-1+qd2.*t10.*t11.*t12.*t15.*1.8252e-3+qd5.*t11.*t12.*t13.*t15.*1.8252e-3-qd7.*t11.*t12.*t13.*t14.*(3.0./1.0e2)+qd1.*t2.*t3.*t4.*t10.*t11.*9.36e-3-qd1.*t2.*t5.*t6.*t8.*t14.*1.8252e-3+qd1.*t3.*t4.*t8.*t9.*t11.*3.6504e-3+qd3.*t2.*t3.*t7.*t10.*t11.*9.126e-3+qd1.*t5.*t6.*t7.*t9.*t10.*1.8252e-3-qd2.*t2.*t4.*t7.*t10.*t13.*9.126e-3-qd2.*t2.*t3.*t8.*t11.*t13.*9.126e-3+qd1.*t3.*t4.*t9.*t11.*t14.*9.126e-3+qd2.*t3.*t4.*t7.*t12.*t14.*9.126e-3+qd1.*t3.*t4.*t9.*t11.*t15.*3.6504e-3+qd2.*t3.*t4.*t7.*t12.*t15.*1.8252e-3-qd1.*t2.*t5.*t6.*t14.*t16.*1.8252e-3-qd1.*t3.*t6.*t10.*t12.*t13.*1.8252e-3+qd2.*t2.*t3.*t11.*t13.*t14.*3.6504e-3+qd5.*t2.*t3.*t10.*t11.*t14.*1.8252e-3+qd1.*t2.*t5.*t8.*t14.*t16.*1.8252e-3+qd1.*t5.*t7.*t9.*t10.*t14.*9.126e-3-qd1.*t2.*t6.*t8.*t14.*t16.*1.8252e-3+qd1.*t5.*t7.*t9.*t10.*t15.*3.6504e-3-qd1.*t6.*t7.*t9.*t10.*t14.*9.126e-3-qd3.*t2.*t4.*t8.*t14.*t16.*1.8252e-3-qd3.*t4.*t7.*t9.*t10.*t14.*9.126e-3-qd1.*t4.*t7.*t11.*t12.*t13.*1.8252e-3-qd1.*t6.*t7.*t9.*t10.*t15.*3.6504e-3-qd3.*t3.*t8.*t9.*t11.*t14.*9.126e-3-qd3.*t4.*t7.*t9.*t10.*t15.*3.6504e-3+qd2.*t4.*t7.*t9.*t13.*t14.*9.36e-3-qd3.*t3.*t8.*t9.*t11.*t15.*3.6504e-3+qd2.*t4.*t8.*t9.*t13.*t14.*9.126e-3-qd5.*t3.*t7.*t9.*t11.*t15.*1.8252e-3+qd2.*t4.*t8.*t9.*t13.*t15.*3.6504e-3+qd7.*t3.*t7.*t9.*t11.*t14.*(3.0./1.0e2)-qd1.*t3.*t10.*t12.*t13.*t14.*9.126e-3-qd1.*t3.*t10.*t12.*t13.*t15.*3.6504e-3-qd2.*t5.*t10.*t11.*t12.*t14.*9.126e-3-qd4.*t3.*t10.*t11.*t12.*t14.*9.126e-3+qd1.*t4.*t11.*t12.*t13.*t14.*9.36e-3-qd2.*t5.*t10.*t11.*t12.*t15.*3.6504e-3-qd4.*t3.*t10.*t11.*t12.*t15.*1.8252e-3-qd4.*t2.*t11.*t13.*t14.*t16.*1.8252e-3+qd3.*t7.*t11.*t12.*t13.*t15.*1.8252e-3+qd1.*t2.*t3.*t4.*t7.*t10.*t11.*1.8252e-2-qd1.*t3.*t4.*t7.*t9.*t11.*t14.*9.36e-3-qd1.*t3.*t4.*t8.*t9.*t11.*t14.*1.8252e-2+qd3.*t2.*t3.*t7.*t10.*t11.*t14.*1.8252e-3-qd1.*t3.*t4.*t8.*t9.*t11.*t15.*7.3008e-3-qd1.*t2.*t5.*t6.*t8.*t14.*t16.*1.8252e-3-qd1.*t5.*t6.*t7.*t9.*t10.*t14.*9.126e-3-qd2.*t2.*t4.*t7.*t10.*t13.*t14.*1.8252e-3-qd1.*t5.*t6.*t7.*t9.*t10.*t15.*3.6504e-3-qd2.*t2.*t3.*t8.*t11.*t13.*t14.*1.8252e-3+qd2.*t3.*t7.*t9.*t10.*t11.*t13.*1.8252e-3-qd4.*t2.*t4.*t9.*t10.*t12.*t14.*1.8252e-3+qd2.*t2.*t7.*t9.*t11.*t12.*t14.*1.8252e-3+qd1.*t3.*t6.*t10.*t12.*t13.*t14.*9.126e-3+qd1.*t3.*t6.*t10.*t12.*t13.*t15.*3.6504e-3-qd2.*t2.*t3.*t11.*t13.*t14.*t16.*1.8252e-3+qd1.*t4.*t7.*t11.*t12.*t13.*t14.*9.126e-3+qd1.*t4.*t7.*t11.*t12.*t13.*t15.*3.6504e-3+qd1.*t2.*t3.*t4.*t7.*t10.*t11.*t14.*3.6504e-3-qd2.*t2.*t3.*t4.*t9.*t10.*t12.*t14.*1.8252e-3-qd1.*t2.*t3.*t7.*t9.*t12.*t13.*t14.*3.6504e-3-qd2.*t2.*t5.*t7.*t9.*t11.*t12.*t14.*3.6504e-3-qd4.*t2.*t3.*t7.*t9.*t11.*t12.*t14.*1.8252e-3+qd3.*t2.*t3.*t7.*t10.*t11.*t14.*t16.*1.8252e-3-qd2.*t2.*t4.*t7.*t10.*t13.*t14.*t16.*1.8252e-3-qd2.*t2.*t3.*t8.*t11.*t13.*t14.*t16.*1.8252e-3-qd2.*t3.*t7.*t9.*t10.*t11.*t13.*t14.*9.126e-3-qd2.*t3.*t7.*t9.*t10.*t11.*t13.*t15.*3.6504e-3-qd3.*t2.*t9.*t10.*t11.*t12.*t13.*t14.*1.8252e-3+qd1.*t2.*t3.*t6.*t7.*t9.*t12.*t13.*t14.*3.6504e-3+qd1.*t2.*t3.*t4.*t7.*t10.*t11.*t14.*t16.*3.6504e-3-qd1.*t2.*t4.*t9.*t10.*t11.*t12.*t13.*t14.*3.6504e-3,qd2.*t18.*(-9.126e-4)+qd2.*t2.*t5.*9.126e-3+qd4.*t2.*t3.*9.126e-3+qd2.*t2.*t7.*9.36e-3+qd2.*t2.*t8.*9.126e-3-qd5.*t3.*t12.*1.918252e-1-qd3.*t9.*t13.*(1.9e1./1.0e2)-qd2.*t2.*t5.*t8.*9.126e-3+qd2.*t2.*t5.*t14.*3.6504e-3+qd4.*t2.*t3.*t14.*1.8252e-3-qd3.*t3.*t7.*t12.*1.918252e-1+qd2.*t2.*t8.*t14.*1.8252e-3+qd1.*t4.*t9.*t13.*1.918252e-1-qd2.*t7.*t9.*t10.*1.8252e-3+qd7.*t2.*t10.*t13.*(3.0./1.0e2)-qd3.*t8.*t9.*t13.*1.8252e-3-qd1.*t10.*t11.*t12.*1.918252e-1-qd5.*t7.*t9.*t13.*1.918252e-1+qd2.*t9.*t10.*t14.*9.36e-3+qd5.*t3.*t12.*t15.*1.8252e-3-qd7.*t3.*t12.*t14.*(3.0./1.0e2)-qd3.*t9.*t13.*t14.*9.126e-3+qd4.*t10.*t12.*t13.*(1.9e1./1.0e2)-qd3.*t9.*t13.*t15.*1.8252e-3+qd1.*t3.*t4.*t7.*t12.*(1.9e1./1.0e2)+qd1.*t2.*t3.*t11.*t13.*9.126e-3-qd1.*t2.*t4.*t10.*t13.*9.36e-3-qd2.*t2.*t5.*t8.*t14.*1.8252e-3+qd2.*t5.*t7.*t9.*t10.*1.8252e-3+qd1.*t3.*t4.*t12.*t14.*9.36e-3-qd1.*t4.*t8.*t9.*t13.*1.8252e-3-qd3.*t2.*t7.*t10.*t13.*9.126e-3+qd1.*t5.*t10.*t11.*t12.*1.8252e-3-qd2.*t2.*t5.*t14.*t16.*1.8252e-3-qd4.*t2.*t3.*t14.*t16.*1.8252e-3-qd2.*t3.*t10.*t12.*t13.*1.8252e-3+qd3.*t3.*t7.*t12.*t15.*1.8252e-3-qd1.*t4.*t9.*t13.*t15.*1.8252e-3+qd2.*t2.*t8.*t14.*t16.*1.8252e-3+qd2.*t7.*t9.*t10.*t14.*9.126e-3+qd2.*t7.*t9.*t10.*t15.*3.6504e-3-qd5.*t2.*t10.*t13.*t14.*1.8252e-3+qd3.*t8.*t9.*t13.*t14.*9.126e-3+qd3.*t8.*t9.*t13.*t15.*3.6504e-3+qd1.*t10.*t11.*t12.*t15.*1.8252e-3+qd5.*t7.*t9.*t13.*t15.*1.8252e-3-qd7.*t7.*t9.*t13.*t14.*(3.0./1.0e2)+qd4.*t10.*t12.*t13.*t14.*9.126e-3+qd4.*t10.*t12.*t13.*t15.*1.8252e-3-qd1.*t2.*t4.*t7.*t10.*t13.*9.126e-3-qd1.*t2.*t3.*t8.*t11.*t13.*9.126e-3+qd1.*t3.*t4.*t7.*t12.*t14.*9.126e-3+qd1.*t3.*t4.*t7.*t12.*t15.*1.8252e-3+qd1.*t2.*t3.*t11.*t13.*t14.*3.6504e-3-qd2.*t2.*t5.*t8.*t14.*t16.*1.8252e-3-qd2.*t5.*t7.*t9.*t10.*t14.*9.126e-3+qd1.*t4.*t7.*t9.*t13.*t14.*9.36e-3-qd2.*t5.*t7.*t9.*t10.*t15.*3.6504e-3+qd1.*t4.*t8.*t9.*t13.*t14.*9.126e-3-qd3.*t2.*t7.*t10.*t13.*t14.*1.8252e-3+qd1.*t4.*t8.*t9.*t13.*t15.*3.6504e-3-qd1.*t5.*t10.*t11.*t12.*t14.*9.126e-3-qd1.*t5.*t10.*t11.*t12.*t15.*3.6504e-3+qd2.*t3.*t10.*t12.*t13.*t14.*9.126e-3+qd2.*t3.*t10.*t12.*t13.*t15.*3.6504e-3-qd1.*t2.*t4.*t7.*t10.*t13.*t14.*1.8252e-3-qd1.*t2.*t3.*t8.*t11.*t13.*t14.*1.8252e-3-qd3.*t2.*t3.*t9.*t10.*t12.*t14.*1.8252e-3+qd1.*t3.*t7.*t9.*t10.*t11.*t13.*1.8252e-3+qd1.*t2.*t7.*t9.*t11.*t12.*t14.*1.8252e-3-qd1.*t2.*t3.*t11.*t13.*t14.*t16.*1.8252e-3+qd4.*t2.*t7.*t9.*t12.*t13.*t14.*1.8252e-3-qd3.*t2.*t7.*t10.*t13.*t14.*t16.*1.8252e-3-qd1.*t2.*t3.*t4.*t9.*t10.*t12.*t14.*1.8252e-3-qd1.*t2.*t5.*t7.*t9.*t11.*t12.*t14.*3.6504e-3+qd2.*t2.*t3.*t7.*t9.*t12.*t13.*t14.*3.6504e-3-qd1.*t2.*t4.*t7.*t10.*t13.*t14.*t16.*1.8252e-3-qd1.*t2.*t3.*t8.*t11.*t13.*t14.*t16.*1.8252e-3-qd1.*t3.*t7.*t9.*t10.*t11.*t13.*t14.*9.126e-3-qd1.*t3.*t7.*t9.*t10.*t11.*t13.*t15.*3.6504e-3,qd3.*t2.*9.126e-3+qd1.*t2.*t4.*9.126e-3-qd3.*t2.*t8.*9.126e-3+qd7.*t2.*t7.*(3.0./1.0e2)+qd4.*t7.*t12.*(1.9e1./1.0e2)-qd2.*t9.*t13.*(1.9e1./1.0e2)+qd5.*t9.*t10.*1.918252e-1-qd1.*t2.*t4.*t8.*9.126e-3+qd1.*t3.*t9.*t11.*(1.9e1./1.0e2)-qd2.*t3.*t7.*t12.*1.918252e-1-qd3.*t2.*t8.*t14.*1.8252e-3-qd5.*t2.*t7.*t14.*1.8252e-3+qd3.*t7.*t9.*t10.*1.8252e-3-qd2.*t8.*t9.*t13.*1.8252e-3+qd3.*t2.*t14.*t16.*1.8252e-3+qd4.*t7.*t12.*t14.*9.126e-3-qd2.*t9.*t13.*t14.*9.126e-3+qd4.*t7.*t12.*t15.*1.8252e-3-qd2.*t9.*t13.*t15.*1.8252e-3-qd5.*t9.*t10.*t15.*1.8252e-3+qd7.*t9.*t10.*t14.*(3.0./1.0e2)-qd1.*t2.*t4.*t8.*t14.*1.8252e-3+qd1.*t4.*t7.*t9.*t10.*1.8252e-3+qd1.*t3.*t8.*t9.*t11.*1.8252e-3-qd2.*t2.*t7.*t10.*t13.*9.126e-3+qd1.*t2.*t4.*t14.*t16.*1.8252e-3+qd1.*t3.*t9.*t11.*t14.*9.126e-3+qd1.*t3.*t9.*t11.*t15.*1.8252e-3+qd2.*t3.*t7.*t12.*t15.*1.8252e-3-qd3.*t2.*t8.*t14.*t16.*1.8252e-3-qd3.*t7.*t9.*t10.*t14.*9.126e-3-qd1.*t7.*t11.*t12.*t13.*1.918252e-1-qd3.*t7.*t9.*t10.*t15.*3.6504e-3+qd2.*t8.*t9.*t13.*t14.*9.126e-3+qd2.*t8.*t9.*t13.*t15.*3.6504e-3+qd1.*t2.*t3.*t7.*t10.*t11.*9.126e-3-qd1.*t2.*t4.*t8.*t14.*t16.*1.8252e-3-qd1.*t4.*t7.*t9.*t10.*t14.*9.126e-3-qd1.*t3.*t8.*t9.*t11.*t14.*9.126e-3-qd1.*t4.*t7.*t9.*t10.*t15.*3.6504e-3-qd1.*t3.*t8.*t9.*t11.*t15.*3.6504e-3-qd2.*t2.*t7.*t10.*t13.*t14.*1.8252e-3-qd4.*t2.*t9.*t10.*t12.*t14.*1.8252e-3+qd1.*t7.*t11.*t12.*t13.*t15.*1.8252e-3+qd1.*t2.*t3.*t7.*t10.*t11.*t14.*1.8252e-3-qd2.*t2.*t3.*t9.*t10.*t12.*t14.*1.8252e-3-qd2.*t2.*t7.*t10.*t13.*t14.*t16.*1.8252e-3+qd1.*t2.*t3.*t7.*t10.*t11.*t14.*t16.*1.8252e-3-qd1.*t2.*t9.*t10.*t11.*t12.*t13.*t14.*1.8252e-3,qd4.*t2.*9.126e-3-qd5.*t12.*1.918252e-1+qd4.*t18.*9.126e-4+qd2.*t2.*t3.*9.126e-3+qd3.*t7.*t12.*(1.9e1./1.0e2)+qd5.*t12.*t15.*1.8252e-3-qd7.*t12.*t14.*(3.0./1.0e2)+qd2.*t2.*t3.*t14.*1.8252e-3+qd1.*t4.*t7.*t12.*(1.9e1./1.0e2)+qd1.*t2.*t11.*t13.*9.126e-3+qd3.*t7.*t12.*t14.*9.126e-3-qd4.*t2.*t14.*t16.*1.8252e-3+qd2.*t10.*t12.*t13.*(1.9e1./1.0e2)+qd3.*t7.*t12.*t15.*1.8252e-3-qd1.*t3.*t10.*t11.*t12.*(1.9e1./1.0e2)-qd2.*t2.*t3.*t14.*t16.*1.8252e-3+qd1.*t4.*t7.*t12.*t14.*9.126e-3+qd1.*t4.*t7.*t12.*t15.*1.8252e-3+qd1.*t2.*t11.*t13.*t14.*1.8252e-3+qd2.*t10.*t12.*t13.*t14.*9.126e-3+qd2.*t10.*t12.*t13.*t15.*1.8252e-3-qd3.*t2.*t9.*t10.*t12.*t14.*1.8252e-3-qd1.*t3.*t10.*t11.*t12.*t14.*9.126e-3-qd1.*t3.*t10.*t11.*t12.*t15.*1.8252e-3-qd1.*t2.*t11.*t13.*t14.*t16.*1.8252e-3-qd1.*t2.*t4.*t9.*t10.*t12.*t14.*1.8252e-3+qd2.*t2.*t7.*t9.*t12.*t13.*t14.*1.8252e-3-qd1.*t2.*t3.*t7.*t9.*t11.*t12.*t14.*1.8252e-3,qd7.*t2.*(3.0./1.0e2)-qd4.*t12.*1.918252e-1-qd5.*t18.*9.126e-4-qd2.*t3.*t12.*1.918252e-1+qd3.*t9.*t10.*1.918252e-1+qd4.*t12.*t15.*1.8252e-3+qd1.*t4.*t9.*t10.*1.918252e-1-qd3.*t2.*t7.*t14.*1.8252e-3-qd2.*t7.*t9.*t13.*1.918252e-1+qd2.*t3.*t12.*t15.*1.8252e-3-qd1.*t11.*t12.*t13.*1.918252e-1-qd3.*t9.*t10.*t15.*1.8252e-3-qd1.*t2.*t4.*t7.*t14.*1.8252e-3+qd1.*t3.*t7.*t9.*t11.*1.918252e-1-qd1.*t4.*t9.*t10.*t15.*1.8252e-3-qd2.*t2.*t10.*t13.*t14.*1.8252e-3+qd2.*t7.*t9.*t13.*t15.*1.8252e-3+qd1.*t11.*t12.*t13.*t15.*1.8252e-3+qd1.*t2.*t3.*t10.*t11.*t14.*1.8252e-3-qd1.*t3.*t7.*t9.*t11.*t15.*1.8252e-3,0.0,qd5.*t2.*(3.0./1.0e2)+qd3.*t2.*t7.*(3.0./1.0e2)-qd4.*t12.*t14.*(3.0./1.0e2)+qd1.*t2.*t4.*t7.*(3.0./1.0e2)+qd2.*t2.*t10.*t13.*(3.0./1.0e2)-qd2.*t3.*t12.*t14.*(3.0./1.0e2)+qd3.*t9.*t10.*t14.*(3.0./1.0e2)-qd1.*t2.*t3.*t10.*t11.*(3.0./1.0e2)+qd1.*t4.*t9.*t10.*t14.*(3.0./1.0e2)-qd2.*t7.*t9.*t13.*t14.*(3.0./1.0e2)-qd1.*t11.*t12.*t13.*t14.*(3.0./1.0e2)+qd1.*t3.*t7.*t9.*t11.*t14.*(3.0./1.0e2)];
