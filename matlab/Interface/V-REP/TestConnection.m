% TEST CONNECTION WITH VREP

clear all
close all
clc



v = VREP();
arm = VREP_arm(v, 'LBR4p','fmt','%s_joint%d');
    