%% if you generate Jacob0 with rpy active is gonna take a big take to compute


close all
clear variables
clc

%% Instantiate a |CodeGenerator| class object

% change the function to change the robot for generating mex matrix
%rob = MdlJaco();
%rob = MdlJacoDH();
%rob = MdlLBR4p();
%rob = MdlLBR4pSimple();
%genkinonly = true;

rob = MdlLBR4pSimple();
genkinonly = false;
cGen = CodeGenerator(rob,'mex','genmfun','genmex');
if(genkinonly)
   %with this function i build only the kinematic symbolic rapresentation
   cGen.genkinematic('norpy','noeul',false);
else
   %with this function i build all the symbolic rapresentation of the robot
   cGen.geneverything('norpy','noeul',false);
end
% add the generated class to matlab path
addpath(cGen.basepath);

% create an empty file to advise that a symbolic version of the robot exist
rob_name = rob.name;
rob_name(rob_name==' ')=[];
fid = fopen( strcat(cGen.basepath,'/',rob_name,'_done.m'), 'wt' );
