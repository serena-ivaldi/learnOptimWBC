%% OVERWRITING SOME OF THE PARAMETERS CONTAINED IN gains.m WHEN USING FSM
if strcmpi(SM.SM_TYPE, 'STANDUP')
    
    CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                                       % of the center of mass are smoothed internally 
    CONFIG.SMOOTH_DES_Q        = 0;    % If equal to one, the desired streamed values 
                                       % of the postural tasks are smoothed internally    
  
     %% State parameters
     sm.stateAt0                   = 1;
     sm.tBalancing                 = 3;
  
     reg.pinvTol     = 1e-5;
     reg.pinvDamp    = 1; 
     reg.pinvDampVb  = 1e-3;
     reg.HessianQP   = 1e-2;
     reg.impedances  = 0.1;
     reg.dampings    = 0;
     
     %% Contact constraints    
     sat.torque                =  60;
     phys.legSize              = [ -0.025  0.025 ;        % xMin, xMax
                                   -0.005  0.005];        % yMin, yMax 
     gain.legSize              = [ -0.025  0.05 ;         % xMin, xMax
                                   -0.025  0.025];        % yMin, yMax 
                                       
     addpath('../../../../utilityMatlabFunctions/')
     [ConstraintsMatrixLegs,bVectorConstraintsLegs] = constraints...
     (forceFrictionCoefficient,numberOfPoints,torsionalFrictionCoefficient,gain.legSize,fZmin);

     gain.PCOM     =    [50   50  50;    % state ==  1  BALANCING ON THE LEGS
                         50   50  50;    % state ==  2  MOVE COM FORWARD
                         50   50  50;    % state ==  3  LOOKING FOR CONTACT
                         50   50  50;    % state ==  4  TWO FEET BALANCING
                         50   50  50;    % state ==  5  MOVE ARMS FORWARD      
                         50   50  50;    % state ==  6  LOOKING FOR CONTACT
                         50   50  50;    % state ==  7  SITTING DOWN 
                         50   50  50];   % state ==  8  BALANCING ON THE LEGS

     gain.ICOM              = gain.PCOM*0;
     gain.DCOM              = 2*sqrt(gain.PCOM);
     gain.PAngularMomentum  = 1 ;
     gain.DAngularMomentum  = 2*sqrt(gain.PAngularMomentum);

                         %   TORSO %%      LEFT ARM    %%      RIGHT ARM   %%         LEFT LEG           %%         RIGHT LEG          %% 
     gain.impedances  = [10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  1  BALANCING ON THE LEGS
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  2  MOVE COM FORWARD
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  3  LOOKING FOR CONTACT
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  4  TWO FEET BALANCING
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  5  MOVE ARMS FORWARD 
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  6  LOOKING FOR CONTACT
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50;   % state ==  7  SITTING DOWN
                         10   30   20, 10   10    10    8, 10   10    10    8, 30   50   30    60    50  50, 30   50   30    60    50  50];  % state ==  8  BALANCING ON THE LEGS
                         
     gain.impedances(3,:) = gain.impedances(3,:)./2;      
                     
end

% set this variable to TRUE if you want iCub also sits down after standing up
sm.alsoSitDown                 = false;

sm.jointsAndCoMSmoothingTimes = [1;    % state ==  1  BALANCING ON THE LEGS
                                 2.5;  % state ==  2  MOVE COM FORWARD
                                 2;    % state ==  3  TWO FEET BALANCING
                                 4;    % state ==  4  LIFTING UP  
                                 2     % state ==  5  MOVE ARMS FORWARD
                                 4     % state ==  6  LOOKING FOR CONTACT
                                 3     % state ==  7  SITTING DOWN
                                 1];   % state ==  8  BALANCING ON THE LEGS 
                             
                                  %Hip pitch  %Hip roll  %Knee     %Ankle pitch  %Shoulder pitch  %Shoulder roll  %Shoulder yaw   %Elbow   %Torso pitch                        
sm.joints.standUpPositions     = [0.0000      0.0000     0.0000    0.0000        0.0000           0.4363          0.0000          0.0000   0.0000;   % state ==  1  THIS REFERENCE IS NOT USED
                                  1.5402      0.1594    -1.7365   -0.2814       -1.6455           0.4363          0.5862          0.2473   0.4363;   % state ==  2  MOVE COM FORWARD
                                  1.1097      0.0122    -0.8365   -0.0714       -1.4615           0.4363          0.1545          0.2018   0.0611;   % state ==  3  TWO FEET BALANCING
                                  0.2094      0.1047    -0.1745   -0.0349       -0.0873           0.2563          0.5862          0.2473   0.0000;   % state ==  4  LIFTING UP
                                  0.2094      0.1047    -0.1745   -0.0349       -1.4615           0.4363          0.5862          0.2473   0.0000;   % state ==  5  MOVE ARMS FORWARD
                                  1.5402      0.0122    -1.7365   -0.0714       -1.4615           0.4363          0.1545          0.2018   0.0000;   % state ==  6  LOOKING FOR CONTACT
                                  1.5402      0.0122    -1.7365   -0.2814       -1.6455           0.4363          0.5862          0.2473   0.0611;   % state ==  7 SITTING DOWN
                                  1.5402      0.0122    -1.7365   -0.2814       -1.4615           0.4363          0.1545          0.2018   0.4363];  % state ==  8  BALANCING ON THE LEGS
                                           
sm.CoM.standUpDeltaCoM         = [0.0     0.0   0.0;       % state ==  1  THIS REFERENCE IS NOT USED
                                  0.1567  0.0   0.0;       % state ==  2  MOVE COM FORWARD
                                  0.08    0.0   0.0;       % state ==  3  TWO FEET BALANCING
                                  0.03    0.0   0.20;      % state ==  4  LIFTING UP
                                  0.0     0.0   0.0;       % state ==  5  MOVE ARMS FORWARD
                                 -0.03    0.0  -0.22;      % state ==  6  LOOKING FOR CONTACT
                                 -0.03    0.0  -0.0;       % state ==  7 SITTING DOWN
                                 -0.1567  0.0   0.0];      % state ==  8  BALANCING ON THE LEGS                           
                                   
sm.LwrenchThreshold    = [0;    % state ==  1  THIS REFERENCE IS NOT USED
                          75;   % state ==  2  MOVE COM FORWARD
                          130;  % state ==  3  TWO FEET BALANCING
                          0;    % state ==  4  THIS REFERENCE IS NOT USED
                          0;    % state ==  5  THIS REFERENCE IS NOT USED
                          140;   % state ==  6  LOOKING FOR CONTACT
                          0     % state ==  7  THIS REFERENCE IS NOT USED
                          0];   % state ==  8  THIS REFERENCE IS NOT USED 
                     
sm.RwrenchThreshold    = [0     % state ==  1  THIS REFERENCE IS NOT USED
                          75;   % state ==  2  MOVE COM FORWARD
                          130;  % state ==  3  TWO FEET BALANCING
                          0;    % state ==  4  THIS REFERENCE IS NOT USED
                          0;    % state ==  5  THIS REFERENCE IS NOT USED
                          140;   % state ==  6  LOOKING FOR CONTACT
                          0     % state ==  7  THIS REFERENCE IS NOT USED
                          0];   % state ==  8  THIS REFERENCE IS NOT USED  
                     
% external forces at arms threshold                    
sm.RArmThreshold      = [10    % state ==  1  BALANCING ON THE LEGS
                          0;   % state ==  2  THIS REFERENCE IS NOT USED
                          0;   % state ==  3  THIS REFERENCE IS NOT USED
                          0;   % state ==  4  THIS REFERENCE IS NOT USED
                        -10;   % state ==  5  MOVE ARMS FORWARD
                          0;   % state ==  6  THIS REFERENCE IS NOT USED
                          0    % state ==  7  THIS REFERENCE IS NOT USED
                          0];  % state ==  8  THIS REFERENCE IS NOT USED 
                      
sm.LArmThreshold      = [10    % state ==  1  BALANCING ON THE LEGS
                          0;   % state ==  2  THIS REFERENCE IS NOT USED
                          0;   % state ==  3  THIS REFERENCE IS NOT USED
                          0;   % state ==  4  THIS REFERENCE IS NOT USED
                        -10;   % state ==  5  MOVE ARMS FORWARD
                          0;   % state ==  6  THIS REFERENCE IS NOT USED
                          0    % state ==  7  THIS REFERENCE IS NOT USED
                          0];  % state ==  8  THIS REFERENCE IS NOT USED 
                      
                      