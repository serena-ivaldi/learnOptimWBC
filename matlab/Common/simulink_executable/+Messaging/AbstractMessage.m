classdef (Abstract) AbstractMessage < handle
   properties
       input2simulink      = 'inputData.mat'
       outputfromsimulink  = 'simulationResults.mat'
   end
   methods(Abstract = true)
      Pack(obj,controller,params);
      Unpack(obj,controller,params); % for Unpack is mandatory to return [t, q, qd]
      StoreFromSimulink(obj,results);
   end
    
end