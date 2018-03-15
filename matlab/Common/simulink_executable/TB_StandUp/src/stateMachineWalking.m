function [CoMDes,qDes,constraints, currentState,impedances,w_H_b] = ...
         stateMachineWalking(connection,CoM_0,q0,w_CoM,CoMIn,qIn,constraintsIn,wrench_rightFoot,wrench_leftFoot,...
                             l_sole_H_b,r_sole_H_b,sm,gain)
    %#codegen    
    persistent state;
    persistent fixedLink;
    persistent w_H_fixedLink;
    
    if isempty(state) || isempty(fixedLink) || isempty(w_H_fixedLink) 
        state         = sm.stateAt0;
        fixedLink     = 1;
        w_H_fixedLink = eye(4);
    end
        
%    w_H_fixedLink  = eye(4);
%     
%    fixedLink      =  1;  %1 = left, 2 = right
    
    CoMDes      = CoM_0;
    constraints = [1; 1];
    qDes        = q0;
    impedances  = gain.impedances(1,:);
    
    %% STATES
    % 1 - initial state (waiting for references)
    % 2 - 2 feet CoM Tracking
    % 3 - Left foot balancing
    % 4 - Right foot balancing
    
    if fixedLink == 1
        w_H_b   =  w_H_fixedLink * l_sole_H_b;
    else 
        w_H_b   =  w_H_fixedLink * r_sole_H_b;
    end
    
    if state == 1
        %waiting for com reference
        CoMDes     = CoM_0;
            
        if norm(CoM_0 - CoMIn) > eps && connection
            state = 2;
        end
        
    elseif state == 2
        
        CoMDes = CoMIn;
        qDes   = qIn;
        impedances  = gain.impedances(state,:); 
        CoMError    = CoMDes - w_CoM;
        
        if ~any(constraintsIn - [1; 0]) ...
                && norm(CoMError(2)) < sm.com.threshold
            constraints = [1; 0]; %right foot is no longer a constraints
            fixedLink = 1;
            w_H_fixedLink = w_H_b / l_sole_H_b;
            state = 3;
            
        elseif ~any(constraintsIn - [0; 1]) ...
                && norm(CoMError(2)) < sm.com.threshold
            constraints = [0; 1]; %left foot is no longer a constraints
            state     = 4;        
            fixedLink = 2;
            w_H_fixedLink = w_H_b / r_sole_H_b;
           
        end
            
    elseif state == 3
        % Left foot balancing
        constraints = [1; 0];

        CoMDes = CoMIn;
        qDes   = qIn;
        impedances  = gain.impedances(state,:);
        
        if wrench_rightFoot(3) > sm.wrench.thresholdContactOn
            state = 2;
        end
        
    elseif state == 4
        % Right foot balancing
        constraints = [0; 1];
        
        CoMDes = CoMIn;
        qDes   = qIn;
        impedances  = gain.impedances(state,:);
        
        if wrench_leftFoot(3) > sm.wrench.thresholdContactOn
            state = 2;
        end
    end
       
    currentState = state;
    
end
