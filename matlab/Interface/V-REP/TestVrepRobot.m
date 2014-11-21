close all
clear all
clc


target_link=[7];
P = 0;
[LBR4p] = MDL_LBR4p(target_link,P);
LBR4p.teach();



v = VAREP();
arm = VAREP_arm(v, 'LBR4p','fmt','%s_joint%d');
arm.teach();



