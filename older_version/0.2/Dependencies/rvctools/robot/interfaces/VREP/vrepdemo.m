% open a connection to the VREP simulator
%
% Notes::
% - the VREP constructor needs to know where V-REP is installed.
% - This can come from the environment variable VREP
% - Or you can edit this file to something like
%    vrep=VREP('/Applications/V-REP_PRO_EDU_V3_1_3_rev2b_Mac');

%%begin

% create a connection to a running instance of V-REP
vrep = VREP();

% add the human figure Bill
bill = vrep.loadmodel('people/Walking Bill');

% get the initial pose of Bill
T = bill.getpose()

% now we can change his position
bill.setpos([0.1, 0.2, 0]);

% make him turn to his right
bill.setorient([0 0 -pi/4])

% make him lean forward a bit
bill.setorient([0 pi/8 0])

% now set him back to his initial position and orientation
bill.setpose(T);

% now we will setup some nested loops to make him shuffle around a square
orient = [0 0 0];
for j=1:4
    step = [0.05 0 0];
    bill.setorient(orient);
    % move forward
    for i=1:30
        % move relative to current pose
        bill.setpos(step, bill);
    end
    % turn at right angles
    orient(3) = orient(3) + pi/2;
end

% remove the model from the scene
bill.remove();
% close the connection to the VREP simulator
clear vrep
% 
% vrep.loadscene('scene1.ttt', 'local');
% camera = vrep.camera('Vision_sensor');
% vrep.simstart();
% im = camera.grab();
% 
% for i=1:20
%     camera.setpos([0 0 0.05], camera);
%     camera.grab();
%     drawnow
% end
         
%         % stop the simulation
%          camera.setpose(T);
%          R = camera.getorient();

%          vrep = VREP();
%          arm = vrep.arm('IRB140');
%          q = arm.getq();
%          arm.setq(zeros(1,6));
%          arm.setpose(T);  % set pose of base