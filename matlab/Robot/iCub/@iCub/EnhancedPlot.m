function EnhancedPlot(obj,q,param)
    %VISUALIZESIMULATION_IDYNTREE calls iDyntree visualizer to simulate robot
    %                             movements.
    %
    % figureCont = VISUALIZESIMULATION_IDYNTREE(chi,CONFIG) takes as input the
    % robot state, chi, and the configuration parameters.
    % The output is a counter for the automatic correction of figures numbers 
    % in case a new figure is added.
    %
    % Author : Gabriele Nava (gabriele.nava@iit.it)
    % Genova, January 2017
    %

    % ------------Initialization----------------
    %% initial parameters
    mdlLdr     = obj.mdlLdr;
    model      = mdlLdr.model();
    
    % visualization time
    init_time  = 1;
    end_time   = length(q(1,:));

    % open the visualizer
    viz   = iDynTree.Visualizer();
    viz.init();
    viz.addModel(model,obj.modelName);
    viz.draw();

    %% Setup environment and lights     
    % disable environmental features     
    env = viz.enviroment();        
    env.setElementVisibility('root_frame',false);        

    % set lights
    sun = viz.enviroment().lightViz('sun');         
    sun.setDirection(obj.lightDir);

    % set camera     
    cam = viz.camera();     
    cam.setPosition(iDynTree.Position(obj.setPos(1),obj.setPos(2),obj.setPos(3)));     
    cam.setTarget(iDynTree.Position(obj.setCamera(1),obj.setCamera(2),obj.setCamera(3)));           

    %% Robot simulation
    for i=init_time:end_time

        tic
        jointPos = iDynTree.JointPosDoubleArray(model);
        joints   = q(:,i);
        jointPos.fromMatlab(joints);

        % compute the world_H_base that correspond to the specified joints 
        odom = iDynTree.SimpleLeggedOdometry();
        odom.setModel(model);
        odom.updateKinematics(jointPos);

        if sum(param.feet_on_ground) == 2

            odom.init('l_sole','l_sole');

        elseif param.feet_on_ground(1) == 1 && param.feet_on_ground(2) == 0

            odom.init('l_sole','l_sole');

        elseif param.feet_on_ground(1) == 0 && param.feet_on_ground(2) == 1

            odom.init('r_sole','r_sole');
        end

        viz.modelViz(0).setPositions(odom.getWorldLinkTransform(model.getDefaultBaseLink()),jointPos);
        viz.draw();
        t = toc;
        pause(max(0,0.01-t))
    end

    pause()





end