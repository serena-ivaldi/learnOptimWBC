function [w_H_b,constraints,impedances,kpCom,kdCom,currentState,jointsAndCoMSmoothingTime,qjDes,CoM_Des] = ...
          stateMachineStandUp(qjRef,CoM,CoM_0,l_sole_H_b,l_upper_leg_contact_H_b,t,gain,sm,Lwrench,Rwrench,LArmWrench,RArmWrench,useExtArmForces)
      
    persistent state;
    persistent tSwitch;
    persistent w_H_fixedLink;
    persistent CoMprevious;

    if isempty(state) || isempty(tSwitch) || isempty(w_H_fixedLink) || isempty(CoMprevious)
        state         = sm.stateAt0;
        tSwitch       = 0;
        w_H_fixedLink = l_sole_H_b/l_upper_leg_contact_H_b;
        CoMprevious   = CoM_0;
    end

    constraints               = [1;1];
    w_H_b                     = eye(4);
    qjDes                     = qjRef;
    CoM_Des                   = CoM_0;
    jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
    
    %% BALANCING ON THE LEGS
    if state == 1 
        
        w_H_b                     =  w_H_fixedLink * l_upper_leg_contact_H_b;
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        % after tBalancing time, start moving CoM forward. If useExtArmForces
        % is enbabled, wait for external help before lifting up.
        if useExtArmForces == 1
            
            if t > sm.tBalancing && RArmWrench(1) > sm.RArmThreshold(state) && LArmWrench(1) > sm.LArmThreshold(state)
                state = 2;   
                tSwitch = t;
            end
        else
            if t > sm.tBalancing
                state = 2;                     
                tSwitch  = t;
            end
        end
    end

    %% MOVE COM FORWARD
    if state == 2 
        
        w_H_b                =  w_H_fixedLink * l_upper_leg_contact_H_b;
        
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        
        tDelta                    = t-tSwitch;
        CoM_Des                   = CoM_0 + transpose(sm.CoM.standUpDeltaCoM(state,:));
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        if (Lwrench(3)+Rwrench(3)) > (sm.LwrenchThreshold(state) + sm.RwrenchThreshold(state)) && tDelta >1.5
            state           = 3;
            w_H_fixedLink   = w_H_fixedLink * l_upper_leg_contact_H_b/l_sole_H_b;
            tSwitch         = t;
            CoMprevious     = CoM;
        end
        
    end

    %% TWO FEET BALANCING
    if state == 3 
        
        w_H_b                =  w_H_fixedLink * l_sole_H_b;

        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));
        tDelta                    = t-tSwitch;
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        
        if Lwrench(3) > sm.LwrenchThreshold(state) &&  Rwrench(3) > sm.RwrenchThreshold(state) && tDelta > 1
            state       = 4;
            tSwitch     = t;
            CoMprevious = CoM;
        end
    end
    
    %% LIFTING UP
    if state == 4 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
          
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
        
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));    
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        tDelta                    = t-tSwitch;
        
        if tDelta > 2.5 && sm.alsoSitDown
            state        = 5;
            tSwitch      = t;
            CoMprevious  = CoM_Des;
        end
                
    end
    
    %% MOVE ARMS FORWARD
    if state == 5 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
          
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
         
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));        
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        tDelta                    = t-tSwitch;
        
        if tDelta > 2.5 && RArmWrench(3) < sm.RArmThreshold(state) && LArmWrench(3) < sm.LArmThreshold(state)            
            state        = 6;
            tSwitch      = t;
            CoMprevious  = CoM;            
        end
                           
    end
    
    %% LOOKING FOR CONTACT
    if state == 6 
        
        w_H_b      =  w_H_fixedLink * l_sole_H_b;
          
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
          
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));         
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        tDelta                    = t-tSwitch;
        
        if (Lwrench(3)+Rwrench(3)) < (sm.LwrenchThreshold(state) + sm.RwrenchThreshold(state)) && tDelta > 4            
            state           = 7;
            tSwitch         = t;
            CoMprevious     = CoM; 
            w_H_fixedLink   = w_H_fixedLink * l_sole_H_b/l_upper_leg_contact_H_b;            
        end
                           
    end
    
    %% SITTING DOWN
    if state == 7
        
        w_H_b      =  w_H_fixedLink * l_upper_leg_contact_H_b;
          
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
          
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));         
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
        tDelta                    = t-tSwitch;
        
        if  tDelta > 10           
            state        = 8;
            CoMprevious  = CoM;            
        end
                           
    end
    
    %% BALACING ON THE LEGS
    if state == 8
        
        w_H_b      =  w_H_fixedLink * l_upper_leg_contact_H_b;
        
        % setup new desired position for some joints: remapper
        qjDes([18 19 21 22]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([12 13 15 16]) = sm.joints.standUpPositions(state,[1 2 3 4]);
        qjDes([8 9 10 11])   = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes([4 5 6 7])     = sm.joints.standUpPositions(state,[5 6 7 8]);
        qjDes(1)             = sm.joints.standUpPositions(state,9);
          
        CoM_Des                   = CoMprevious + transpose(sm.CoM.standUpDeltaCoM(state,:));         
        jointsAndCoMSmoothingTime = sm.jointsAndCoMSmoothingTimes(state);
                           
    end    

    currentState       = state;
    impedances         = gain.impedances(state,:);
    kpCom              = gain.PCOM(state,:);   
    kdCom              = gain.DCOM(state,:);
    
end
 
   