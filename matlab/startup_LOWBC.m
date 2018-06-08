clear variables
close all
clc


disp('OptWBIpath version 0.2')
% set it true if you want to use a fresh installation of rcvtools (is better to use the version in dependecies)
fresh_rcvtools = false;
% set it true if you want to use a fresh installation of gpstuff  (is better to use the version in dependecies)
fresh_gpstuff  = false;

%% find current location 
p = mfilename('fullpath');
fullpath = fileparts(p);
local_rcvtools_path = fullfile('Dependencies','rvctools','startup_rvc.m');
dep_rcvtools_launcher_path = strcat(fullpath,'/',local_rcvtools_path);
local_gpstuff_path = fullfile('Dependencies','gpstuff','startup.m');
dep_gpstuff_launcher_path = strcat(fullpath,'/',local_gpstuff_path);

%% check for robotics toolbox 
p = path;
k = strfind(p,'rvctools');
if(isempty(k))
    if(~fresh_rcvtools)
      run(dep_rcvtools_launcher_path);
      disp('robotics toolbox is installed')
    else
        disp('before installing OptWBI toolbox install robotics toolbox 9.10')
        disp('<a href ="http://www.petercorke.com/RTB/3">link for the toolbox</a>')
    end
elseif(fresh_rcvtools)  
    %% this part is necessary if you install a fresh version of rcvtools and you do not use the one in dependecies
    % copy the updated file inside the robotics toolbox folder
    disp('updating rcvtools')
    str = which('startup_rvc');
    to = fileparts(str);
    from = fullfile(OptWBIpath,'ChangedFile','rvctool');
    % file to CodeGenerator
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','CodeGenerator.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genccodejacobian.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genccodeJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','gencoriolis.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genjacobian.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genccodejacobian.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmexjacobian.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmexJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmfunjacobian.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genmfunJdot.m'),fullfile(to,'robot','@CodeGenerator'));
    copyfile(fullfile(from,'TO_COPY@CodeGenerator','genslblockjacobian.m'),fullfile(to,'robot','@CodeGenerator'));

    %file to SerialLink
    copyfile(fullfile(from,'TO_COPY@SerialLink','coriolis.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','jacob0.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','jacobn.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','rne.m'),fullfile(to,'robot','@SerialLink'));
    copyfile(fullfile(from,'TO_COPY@SerialLink','SerialLink.m'),fullfile(to,'robot','@SerialLink'));

    %file to robot
    copyfile(fullfile(from,'TO_COPY@robot','eul2r.m'),fullfile(to,'robot'));
    copyfile(fullfile(from,'TO_COPY@robot','tr2rpy.m'),fullfile(to,'robot'));
    disp('robotics toolbox is installed')
end
    

%% check for GPstuff 
p = path;
k = strfind(p,'gpstuff');
if(isempty(k))
   if(~fresh_rcvtools)
        run(dep_gpstuff_launcher_path);
        disp('GPstuff toolbox is installed')
    else
        disp('before installing OptWBI toolbox install robotics toolbox 9.10')
        disp('<a href ="http://www.petercorke.com/RTB/3">link for the toolbox</a>')
    end
elseif(fresh_rcvtools) 
    disp('updating gpstuff')
    %% TOADD file that i have changed in gpstuff
    disp('GPstuff toolbox is installed')
end    
    
%% add path to matlab path
disp('installing LWBC toolbox')
OptWBIpath = fileparts( mfilename('fullpath') );
robotpath = fullfile(OptWBIpath, 'Robot');
addpath(genpath(robotpath));
addpath(fullfile(OptWBIpath, 'Common'));
addpath(fullfile(OptWBIpath, 'Common', 'sampling'));
addpath(fullfile(OptWBIpath, 'Common', 'fitness_functions'));
addpath(fullfile(OptWBIpath, 'Common', 'black_box_functions'));
addpath(fullfile(OptWBIpath, 'Common', 'constraints'));
addpath(fullfile(OptWBIpath, 'Common', 'trajectory_generator'));
addpath(fullfile(OptWBIpath, 'Common', 'preprocessing'));
addpath(fullfile(OptWBIpath, 'Common', 'simulink_executable'));
addpath(fullfile(OptWBIpath, 'Classes'));
addpath(fullfile(OptWBIpath, 'Interface'));
testpath = fullfile(OptWBIpath, 'TestResults');
addpath(testpath);
addpath(fullfile(testpath,'datamat'));
addpath(fullfile(testpath,'configuration_files'));


%     % add frne.c
%     copyfile(fullfile(from,'frne.c'),fullfile(to,'robot','mex'));
disp('installation complete!')


