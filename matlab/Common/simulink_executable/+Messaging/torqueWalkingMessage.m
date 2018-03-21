classdef torqueWalkingMessage < Messaging.AbstractMessage
    
   methods
       %% important!! params has always to be saved for make the code it works 
      function Pack(obj,controller,params)
%           WS = controller.GetWholeSystem();
%           %% Updating the robot position and define the world link
%           WS.SetWorldFrameiCub(params.qjInit,params.dqjInit,params.dx_bInit,params.omega_bInit,params.root_reference_link);
%           WS.ComputeSupportPoly(params);
%           %% collecting data from experiments
%           time = params.tStart:params.sim_step:params.tEnd;
%           controller.simulation_iterator     = 1;
%           controller.simulation_results.tau  =  zeros(length(time),WS.ndof);
%           controller.simulation_results.zmp  =  zeros(length(time),2);
%           controller.simulation_results.xCoM =  zeros(length(time),3);
%           controller.simulation_results.Cop  =  zeros(length(time),4);
%           controller.simulation_results.fc   =  cell(length(time),1);
% 
% 
%           % precompute as time series desidered com and its derivatives
%           data_xCoMDes    =  zeros(length(time),3);
%           data_dxCoMDes   =  zeros(length(time),3);
%           data_ddxCoMDes  =  zeros(length(time),3);
%           index = 1;
%           for ti=time
%               [xCoMDes,dxCoMDes,ddxCoMDes]   = controller.references.GetTraj(1,1,ti);
%               data_xCoMDes(index,:)          = xCoMDes;
%               data_dxCoMDes(index,:)         = dxCoMDes;
%               data_ddxCoMDes(index,:)        = ddxCoMDes;
%               index = index + 1;
%           end
%           
%           ts_xCoMDes    = timeseries(data_xCoMDes,time);
%           ts_dxCoMDes   = timeseries(data_dxCoMDes,time);
%           ts_ddxCoMDes  = timeseries(data_ddxCoMDes,time);
%           % save all the releavnt data for the thread
%           %% inputData.mat has to be saved in common/simulink_executable
%           save(obj.input2simulink,'ts_xCoMDes','ts_dxCoMDes','ts_ddxCoMDes','params');      
            save(obj.input2simulink,'params');
      end
      
      function [q, qd,t] = Unpack(obj,controller,params)
%           %% this line has to be present in every unpack function  
%           load(obj.outputfromsimulink)
%           %% save data 
%           controller.simulation_iterator     = 1;
%           controller.simulation_results.tau  = torque_sim.Data;
%           controller.simulation_results.zmp  = zmp_sim.Data;
%           number_of_dims = ndims(com_pos_sim.Data);
%           if(number_of_dims>2)
%               app_mat = squeeze(com_pos_sim.Data);
%               [row,col] = size(app_mat);
%               if(row<col)
%                   app_mat = app_mat';
%               end
%               controller.simulation_results.xCoM =  app_mat;
%           else
%               controller.simulation_results.xCoM =  com_pos_sim.Data;
%           end
%           controller.simulation_results.LsoleWrench =  left_leg_wrench_sim;
%           controller.simulation_results.RsoleWrench =  right_leg_wrench_sim; 
%           q  = q_sim.Data; % row vectors (TODO check if they are in the right order)
%           qd = qd_sim.Data;% row vectors
%           t  = params.tStart:params.sim_step:params.tEnd;
          
      end
      
      function StoreFromSimulink(obj,results)
%         torque_sim           = results.get('torque_sim');
%         zmp_sim              = results.get('zmp_sim');
%         com_pos_sim          = results.get('com_pos_sim');
%         q_sim                = results.get('q_sim');
%         qd_sim               = results.get('qd_sim');
%         left_leg_wrench_sim  = results.get('left_leg_wrench_sim');
%         right_leg_wrench_sim = results.get('right_leg_wrench_sim');
%         %% simulationResults.mat has to be saved in common/simulink_executable
%         save(obj.outputfromsimulink,'torque_sim','zmp_sim','com_pos_sim','q_sim','qd_sim','left_leg_wrench_sim','right_leg_wrench_sim');
      end
   end
    
end