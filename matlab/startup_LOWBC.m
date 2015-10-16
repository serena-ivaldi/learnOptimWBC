clear variables
close all
clc


disp('OptWBIpath version 0.2')

% check for robotics toolbox 
p = path;
k = strfind(p,'rvctools');
if(isempty(k))
   disp('before installing OptWBI toolbox install robotics toolbox 9.10')
   disp('<a href ="http://www.petercorke.com/RTB/3">link for the toolbox</a>')
else
    disp('robotics toolbox is installed')
    % add path to matlab path
    OptWBIpath = fileparts( mfilename('fullpath') );
    robotpath = fullfile(OptWBIpath, 'Robot');
    addpath(genpath(robotpath));
    addpath(fullfile(OptWBIpath, 'Common'));
    addpath(fullfile(OptWBIpath, 'Classes'));
    addpath(fullfile(OptWBIpath, 'Interface'));
    testpath = fullfile(OptWBIpath, 'TestResults');
    addpath(testpath);
    addpath(fullfile(testpath,'datamat'));
    
    disp('updating rcvtools')
    % copy the updated file inside the robotics toolbox folder
    str = which('startup_rvc');
    to = fileparts(str);
    from = fullfile(OptWBIpath,'ChangedFile','rvctool');
    % file to CodeGenerator
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','CodeGenerator.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genccodeJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','gencoriolis.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmexJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmfunJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    
    % file to SerialLink
    copyfile(fullfile(from,'TO_COPY@SerialLink','coriolis.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','jacobn.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','rne.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','SerialLink.m'),fullfile(to,'robot','@SerialLink'));
    
    % add frne.c
    copyfile(fullfile(from,'frne.c'),fullfile(to,'robot','mex'));
    disp('installation complete!')
end


