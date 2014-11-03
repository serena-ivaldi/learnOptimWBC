clear all 
close all 
clc

% simlple function for test the capabilities of of peter corke simulator  


mdl_puma560
p560.fast = 1;

p560.accel(qz, zeros(1,6), zeros(1,6))

% To be useful for simulation this function must be integrated.  fdyn() uses the
% MATLAB function ode45() to integrate the joint acceleration.  It also allows 
% for a user written function to compute the joint torque as a function of 
% manipulator state.
%
% To simulate the motion of the Puma 560 from rest in the zero angle pose 
% with zero applied joint torques

tic
[t q qd] = p560.nofriction().fdyn(10,@sintorque,qz);
toc

% This can be shown in animation also
clf
p560.plot3d(q,'path','/home/vale/Documents/MatlabToolbox/rvctools/contrib/arte/robots/UNIMATE/puma560')



