% SETSMOOTHINGTIMES given the current state, it assigns the smoothing
%                   time for smoothing tasks and joints references.
%
% FORMAT: [smoothingTime_CoM, smoothingTime_LFoot, smoothingTime_RFoot, smoothingTime_joints] ...
%              = setSmoothingTimes(state,Config) 
%
% INPUT:  - state  = current robot state
%         - Config = structure of user-defined configuration parameters
%
% OUTPUT: - smoothingTime_CoM    = smoothing time for CoM references 
%         - smoothingTime_LFoot  = smoothing time for LFoot references
%         - smoothingTime_RFoot  = smoothing time for RFoot references
%         - smoothingTime_joints = smoothing time for joints references
%         - smoothingTime_LHand  = smoothing time for LHand references
%         - smoothingTime_RHand  = smoothing time for RHand references
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---

function [smoothingTime_CoM, smoothingTime_LFoot, smoothingTime_RFoot, smoothingTime_joints, smoothingTime_LHand, smoothingTime_RHand] = setSmoothingTimes(state,Config)

    % Set smoothing times according to the current state
    smoothingTime_CoM    = Config.smoothingTime_CoM(state);
    smoothingTime_LFoot  = Config.smoothingTime_LFoot(state);
    smoothingTime_RFoot  = Config.smoothingTime_RFoot(state);
    smoothingTime_joints = Config.smoothingTime_joints(state);
    smoothingTime_LHand  = Config.smoothingTime_LHand(state);
    smoothingTime_RHand  = Config.smoothingTime_RHand(state);
end