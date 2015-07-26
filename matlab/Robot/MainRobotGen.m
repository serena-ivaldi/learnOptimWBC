close all
clear all
clc

%% Instantiate a |CodeGenerator| class object

% change the function to change the robot for generating mex matrix
rob = MdlJacoDH();
genkinonly = true;
cGen = CodeGenerator(rob,'mex','genmfun','genmex');
if(genkinonly)
   %with this function i build only the kinematic symbolic rapresentation
   cGen.genkinematic();
else
   %with this function i build all the symbolic rapresentation of the robot
   cGen.geneverything();
end
% add the generated class to matlab path
addpath(cGen.basepath);

% create an empty file to advise that a symbolic version of the robot exist
rob_name = rob.name;
rob_name(rob_name==' ')=[]; 
fid = fopen( strcat(cGen.basepath,'/',rob_name,'_done.m'), 'wt' );


