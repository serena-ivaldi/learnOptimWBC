%% wit this script given an inputData.mat you can use it to debug the messaging
%% function StoreFromSimulink of your messenger implementation

clear variables
close all 
clc

name_simulink_folder  = 'TB_StandUp';
name_simulink_schemes = 'torqueBalancing2012b';
scenario_name         = 'sit_icub_to_optimize_0_1.world';
% just temporary until codyco is updated on every machine
codyco                = 'old'; % old or new depending on your codyco installation (2017 codyco version = old 2018 codyco version = new)
% here i build the class that is responsible of the communication among
% matlab processes
messenger             = Messaging.TB_StandUpMessage();


params.name_simulink_schemes = name_simulink_schemes;
params.codyco                = codyco;
params.messenger             = messenger;
params.scenario_name         = scenario_name;
[params.simulink_schemes_global,params.path_to_local_simscheme] = SimulinkInitializationExperiment(name_simulink_folder,scenario_name,codyco);
threadSimulink