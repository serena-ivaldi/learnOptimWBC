close all
clear all
clc


target_link=[7];
P = 0;
[LBR4p] = MdlLBR4p();
LBR4p.teach();



v = VAREP();
arm = VAREP_arm(v, 'LBR4p','fmt','%s_joint%d');
arm.teach();



