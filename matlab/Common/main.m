clear all
close all
clc



target_link = [3,5,6];
[p560 , P560Model] = MdlPuma560(target_link);
 %p560p =p560.perturb(0.1);