function jacob0 = jacob0(rob,in2)
%% JACOB0 - Jacobian with respect to the base coordinate frame of the LBR4pSimple copy arm. 
% ========================================================================= 
%    
%    J0 = jacob0(rob,q) 
%    J0 = rob.jacob0(q) 
%    
%  Description:: 
%    Given a full set of joint variables the function 
%    computes the robot jacobian with respect to the base frame. 
%    
%  Input:: 
%    q:  7-element vector of generalized coordinates. 
%    Angles have to be given in radians! 
%    
%  Output:: 
%    J0:  [6x7] Jacobian matrix 
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
%    This is an autogenerated function! 
%    Code generator written by: 
%    Joern Malzahn 
%    2012 RST, Technische Universitaet Dortmund, Germany 
%    http://www.rst.e-technik.tu-dortmund.de 
%    
%  See also fkine,jacobn.
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
%    25-Feb-2016 21:42:31

q1 = in2(:,1);
q2 = in2(:,2);
q3 = in2(:,3);
q4 = in2(:,4);
q5 = in2(:,5);
q6 = in2(:,6);
q7 = in2(:,7);
t2 = cos(q1);
t3 = cos(q4);
t4 = sin(q1);
t5 = sin(q3);
t6 = t4.*t5;
t7 = cos(q2);
t8 = cos(q3);
t13 = t2.*t7.*t8;
t9 = t6-t13;
t10 = sin(q2);
t11 = sin(q4);
t12 = sin(q5);
t14 = t3.*t9;
t15 = t2.*t10.*t11;
t16 = t14+t15;
t17 = cos(q5);
t18 = t4.*t8;
t19 = t2.*t5.*t7;
t20 = t18+t19;
t21 = sin(q7);
t22 = cos(q6);
t23 = cos(q7);
t24 = sin(q6);
t25 = t12.*t21;
t27 = t17.*t22.*t23;
t26 = t25-t27;
t28 = t3.*t26;
t29 = t11.*t23.*t24;
t30 = t28+t29;
t31 = t17.*t21;
t32 = t12.*t22.*t23;
t33 = t31+t32;
t34 = t22.*(3.9e1./5.0e2);
t35 = t34+3.9e1./1.0e2;
t36 = t11.*t35;
t37 = t3.*t17.*t24.*(3.9e1./5.0e2);
t38 = t36+t37;
t39 = t11.*t26;
t105 = t3.*t23.*t24;
t40 = t39-t105;
t41 = t10.*t40;
t42 = t8.*t30;
t43 = t5.*t33;
t44 = t42+t43;
t107 = t7.*t44;
t45 = t41-t107;
t46 = t5.*t30;
t118 = t8.*t33;
t47 = t46-t118;
t48 = t5.*t38;
t49 = t8.*t12.*t24.*(3.9e1./5.0e2);
t50 = t48+t49;
t51 = t3.*t35;
t74 = t11.*t17.*t24.*(3.9e1./5.0e2);
t52 = t51-t74+2.0./5.0;
t53 = t10.*t52;
t54 = t8.*t38;
t75 = t5.*t12.*t24.*(3.9e1./5.0e2);
t55 = t54-t75;
t56 = t7.*t55;
t57 = t53+t56;
t58 = t9.*t11;
t93 = t2.*t3.*t10;
t59 = t58-t93;
t60 = t24.*t59;
t61 = t16.*t17;
t62 = t12.*t20;
t63 = t61+t62;
t94 = t22.*t63;
t64 = t60-t94;
t65 = t12.*t16;
t96 = t17.*t20;
t66 = t65-t96;
t67 = t12.*t23;
t68 = t17.*t21.*t22;
t69 = t67+t68;
t70 = t3.*t69;
t83 = t11.*t21.*t24;
t71 = t70-t83;
t72 = t17.*t23;
t85 = t12.*t21.*t22;
t73 = t72-t85;
t76 = t4.*t57;
t77 = t2.*t50;
t78 = t76+t77;
t79 = t11.*t69;
t80 = t3.*t21.*t24;
t81 = t79+t80;
t82 = t10.*t81;
t84 = t8.*t71;
t86 = t5.*t73;
t87 = t84+t86;
t98 = t7.*t87;
t88 = t82-t98;
t89 = t5.*t71;
t139 = t8.*t73;
t90 = t89-t139;
t91 = t4.*t50;
t123 = t2.*t57;
t92 = t91-t123;
t95 = t21.*t64;
t109 = t23.*t66;
t97 = t95-t109;
t99 = t22.*t59;
t100 = t24.*t63;
t101 = t99+t100;
t102 = t23.*t64;
t103 = t21.*t66;
t104 = t102+t103;
t106 = t7.*t52;
t161 = t10.*t55;
t108 = t106-t161;
t110 = t2.*t5;
t111 = t4.*t7.*t8;
t112 = t110+t111;
t113 = t3.*t112;
t130 = t4.*t10.*t11;
t114 = t113-t130;
t115 = t2.*t8;
t132 = t4.*t5.*t7;
t116 = t115-t132;
t117 = t2.*t45;
t119 = t4.*t47;
t120 = t117+t119;
t121 = t78.*t120;
t122 = t2.*t47;
t198 = t4.*t45;
t124 = t122-t198;
t199 = t92.*t124;
t125 = t121-t199;
t126 = t11.*t112;
t127 = t3.*t4.*t10;
t128 = t126+t127;
t129 = t24.*t128;
t131 = t17.*t114;
t133 = t12.*t116;
t134 = t131+t133;
t153 = t22.*t134;
t135 = t129-t153;
t136 = t12.*t114;
t155 = t17.*t116;
t137 = t136-t155;
t138 = t2.*t88;
t140 = t4.*t90;
t141 = t138+t140;
t142 = t78.*t141;
t143 = t2.*t90;
t210 = t4.*t88;
t144 = t143-t210;
t211 = t92.*t144;
t145 = t142-t211;
t146 = t5.*t10.*t11.*t22.*(2.0./5.0);
t147 = t8.*t10.*t12.*t24.*(2.0./5.0);
t148 = t5.*t10.*t17.*t24.*(3.9e1./1.0e2);
t149 = t7.*t11.*t12.*t24.*(3.9e1./1.0e2);
t150 = t3.*t8.*t10.*t12.*t24.*(3.9e1./1.0e2);
t151 = t3.*t5.*t10.*t17.*t24.*(2.0./5.0);
t152 = t146+t147+t148+t149+t150+t151;
t154 = t23.*t135;
t156 = t21.*t137;
t157 = t154+t156;
t158 = t7.*t40;
t159 = t10.*t44;
t160 = t158+t159;
t162 = t45.*t108;
t216 = t57.*t160;
t163 = t162-t216;
t164 = t21.*t135;
t177 = t23.*t137;
t165 = t164-t177;
t166 = t7.*t81;
t167 = t10.*t87;
t168 = t166+t167;
t169 = t88.*t108;
t170 = t22.*t128;
t171 = t24.*t134;
t172 = t170+t171;
t173 = t8.*t11.*t22.*(2.0./5.0);
t174 = t8.*t17.*t24.*(3.9e1./1.0e2);
t175 = t3.*t8.*t17.*t24.*(2.0./5.0);
t222 = t5.*t12.*t24.*(2.0./5.0);
t223 = t3.*t5.*t12.*t24.*(3.9e1./1.0e2);
t176 = t173+t174+t175-t222-t223;
t178 = t11.*t17.*t23.*(3.9e1./1.0e2);
t179 = t3.*t23.*t24.*(3.9e1./5.0e2);
t180 = t11.*t17.*t22.*t23.*(3.9e1./5.0e2);
t224 = t11.*t12.*t21.*(3.9e1./5.0e2);
t225 = t11.*t12.*t21.*t22.*(3.9e1./1.0e2);
t181 = t178+t179+t180-t224-t225;
t182 = t11.*t17.*t21.*(3.9e1./1.0e2);
t183 = t11.*t12.*t23.*(3.9e1./5.0e2);
t184 = t3.*t21.*t24.*(3.9e1./5.0e2);
t185 = t11.*t17.*t21.*t22.*(3.9e1./5.0e2);
t186 = t11.*t12.*t22.*t23.*(3.9e1./1.0e2);
t187 = t182+t183+t184+t185+t186;
t188 = t12.*t21.*(3.9e1./1.0e2);
t189 = t12.*t21.*t22.*(3.9e1./5.0e2);
t190 = t17.*t21.*(3.9e1./5.0e2);
t191 = t12.*t23.*(3.9e1./1.0e2);
t192 = t17.*t21.*t22.*(3.9e1./1.0e2);
t193 = t12.*t22.*t23.*(3.9e1./5.0e2);
t194 = t190+t191+t192+t193;
t195 = t7.*t11;
t196 = t3.*t8.*t10;
t197 = t195+t196;
t200 = t12.*t197;
t201 = t5.*t10.*t17;
t202 = t200+t201;
t203 = t17.*t197;
t212 = t5.*t10.*t12;
t204 = t203-t212;
t205 = t22.*t204;
t206 = t3.*t7;
t213 = t8.*t10.*t11;
t207 = t206-t213;
t208 = t24.*t207;
t209 = t205+t208;
t214 = t21.*t202;
t226 = t23.*t209;
t215 = t214-t226;
t217 = t23.*t202;
t218 = t21.*t209;
t219 = t217+t218;
t220 = t24.*t204;
t227 = t22.*t207;
t221 = t220-t227;
t228 = t188+t189-t17.*t23.*(3.9e1./5.0e2)-t17.*t22.*t23.*(3.9e1./1.0e2);
t229 = t11.*t22;
t230 = t3.*t17.*t24;
t231 = t229+t230;
t232 = t3.*t22;
t236 = t11.*t17.*t24;
t233 = t232-t236;
t234 = t8.*t231;
t241 = t5.*t12.*t24;
t235 = t234-t241;
t237 = t7.*t233;
t238 = t5.*t231;
t239 = t8.*t12.*t24;
t240 = t238+t239;
t242 = t237-t10.*t235;
jacob0 = reshape([-t104.*t125+t97.*t145-t101.*t152,t125.*t157-t145.*t165+t152.*t172,-t125.*t215-t145.*t219-t152.*t221,t104.*t160-t97.*t168-t101.*t242,-t157.*t160+t165.*t168+t172.*t242,t160.*t215+t168.*t219-t221.*t242,t104.*t163-t101.*t176-t97.*(t169-t57.*t168),-t157.*t163+t172.*t176+t165.*(t169-t57.*t168),-t176.*t221+t215.*(t162-t216)+t219.*(t169-t57.*t168),-t47.*t104+t90.*t97-t101.*t240,t47.*t157-t90.*t165+t172.*t240,-t47.*t215-t90.*t219-t221.*t240,-t97.*t181+t104.*t187-t11.*t12.*t24.*t101.*(3.9e1./1.0e2),-t157.*t187+t165.*t181+t11.*t12.*t24.*t172.*(3.9e1./1.0e2),t181.*t219+t187.*t215-t11.*t12.*t24.*t221.*(3.9e1./1.0e2),t40.*t104-t81.*t97-t101.*t233,-t40.*t157+t81.*t165+t172.*t233,t40.*t215+t81.*t219-t221.*t233,t97.*t194-t104.*t228-t17.*t24.*t101.*(3.9e1./1.0e2),-t165.*t194+t157.*t228+t17.*t24.*t172.*(3.9e1./1.0e2),-t194.*t219-t215.*t228-t17.*t24.*t221.*(3.9e1./1.0e2),t33.*t104-t73.*t97-t12.*t24.*t101,-t33.*t157+t73.*t165+t12.*t24.*t172,t33.*t215+t73.*t219-t12.*t24.*t221,t23.*t24.*t97.*(-3.9e1./5.0e2)+t21.*t24.*t104.*(3.9e1./5.0e2),t21.*t24.*t157.*(-3.9e1./5.0e2)+t23.*t24.*t165.*(3.9e1./5.0e2),t21.*t24.*t215.*(3.9e1./5.0e2)+t23.*t24.*t219.*(3.9e1./5.0e2),-t22.*t101-t21.*t24.*t97-t23.*t24.*t104,t22.*t172+t23.*t24.*t157+t21.*t24.*t165,-t22.*t221-t23.*t24.*t215+t21.*t24.*t219,t21.*t97.*(3.9e1./5.0e2)+t23.*t104.*(3.9e1./5.0e2),t23.*t157.*(-3.9e1./5.0e2)-t21.*t165.*(3.9e1./5.0e2),t23.*t215.*(3.9e1./5.0e2)-t21.*t219.*(3.9e1./5.0e2),-t23.*t97+t21.*t104,-t21.*t157+t23.*t165,t21.*t215+t23.*t219,0.0,0.0,0.0,-t99-t100,t172,-t220+t227],[6, 7]);
