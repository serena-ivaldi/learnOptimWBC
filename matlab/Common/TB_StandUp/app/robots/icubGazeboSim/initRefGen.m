CONFIG.DEMO_MOVEMENTS      = true; % Either true or false


CONFIG.TIME_CONTROLLER_SWITCH = params.tswitch;                          
                          
                          
% in this model i do not have the last joint of the arms (wrist prosup i guess)
% standing
%CONFIG.JOINTS = [-20   0  0, -30  30  0  45 , -30  30  0  45 , 25.5   0   0  -8  -5.5  0, 25.5   0   0  -8  -5.5  0]'*(pi/180);
% sitting
%CONFIG.JOINTS = [  13   0  0   -20.0  30.0  0.0  45.0  -20.0  30.0  0.0  45.0   90 0  0 -100 -10.4 0   90 0  0 -100 -10.4 0]'*(pi/180);
%CONFIG.JOINTS = [  70   0  0   -71.0  30.0  0.0  45.0  -71.0  30.0  0.0  45.0   52 0  0 -100 -18 0   52 0  0 -100 -18 0]'*(pi/180);
 CONFIG.JOINTS = params.qfinal; %% final joint position
 CONFIG.JOINTSITING = 

    

