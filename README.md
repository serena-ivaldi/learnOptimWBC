learnOptimWBC
=============

Description
-----------

learnOptimWBC is a toolbox for Matlab providing the implementation of a method for the soff prioritization of multiple tasks control.

Some utility functions are imported from MathWorks (original authors are always acknowledged).


Installation
------------
For the installation process follow the instructions in matlab/INSTALL.txt 

Code Structure
--------------

- *Classes* contains all the classes that define the fuctionalities of the toolbox,
- *Common* contains a many support functions and some routines to generate the reference for the controllers,
- *DesignToolbox* contains a set of functions to design new scenarios and new controllers,
- *Exe* contains functions used in the multi-objective framework,
- *Interface* contains some routine to use v-rep as the dynamic simulator,
- *Robot* contains the robot models and their symoblic rapresentations,
- *TestResults* contains the result of each optimization, the scenarios and the structure of each controller,
- *Interface* contains utility functions.
- *UsefullFile* not used.


How to add a new controller 
--------------------

If you wan to define a new controller given a specific robot you have to assign the parameters in AllStaticParameters.m and UF_StaticParameters.m both in /matlab/DesignToolbox

in AllStaticParameters.m

- *CONTROLLERTYPE*  : string defines the kind of controller = 'UF' for our method or 'GHC' a method used for comparison,
- *subchain1*  : row vector defines the links frame that we want to control on a kinematic chain. example 6 dof robot for an e-e cartesian controller subchain1=[6],
- *[bot1]*  : the robot that we want to use it. pick the model from the /Robot folder using the Mdl***() functions,
- *traj_type = {'cartesian_x','cartesian_rpy','joint'}*  : specify the kind of controllers that we want to use (one for each controller),
- *control_type = {'regulation','tracking'}*  : defines if we want to reach a point or if we want to follow a trajectory (one for each controller),
- *type_of_traj = {'none','sampled','func'}*  : for tracking= 'sampled' if we want to generate a sampled trajectory 'func' for closed form trajectory 
                                                regulation = 'none'  (one for each controller),
- *geometric_path = {'rectilinear','lemniscate','circular', 'none'}*  : defines the kind of trajectory that we want to follow for a tracking task. 'none' for a 									regulation task (one for each controller),
- *time_law = {'exponential','exponential','none'}*  : defines the kind of time law that we want to follow for a tracking task. 'none' for a 							       regulation task (one for each controller),       
- *geom_parameters{1,i}*  : defines the geomtric structure of each task. look inside the function in /common to know which are the parameters that we 			            specify for each trajectory. In the regulation case is necessary to specify only the arrival point (one for each controller),
- *dim_of_task{1,i}*  : is a vector of 0 and 1 that define the dimension that we want to control for each task. 1 = dim active 0 = dim not active (one for each controller),
- *id *  : the name of the controller that has to be specified when we want to use the current controller.


in UF_StaticParameters.m
- *metric*  : cell array of strings. According to the peters 2007 we can define different metric that determines different control structure,      
- *kp*  : row vector that defines the proportional gain  (one for each controller).
 


How to add a new scenario
-----------------------

Creating a new scenario is a trial and error procedure. For the obstacles this toolbox provides a global variable called 'G_OB'. The obstacle supported in this library are of the kind 'wall' or the kind 'repulsive point'. The class 'obstacle' contains all the information to set up properly obstacles to design new scenario.
The script DesignScene.m is a sandbox where the can be used to visualize the position in space of the robot and the obstacles and plot the trajectories designed previously.
As we said the scenario design phase is a time consuming operation but in our knowledge this is a common issue for every simulator where creating new scenario can be a painfull task.
To start give a look at /matlab/TestResults/scenarios where is possible to find some simple scenarios.


HOw to run an experiment
--------------

given a scenario and a controller we can run a simulation (with /matlab/Exe/MainExec.m) or learn the parameters in the controller (with /matlab/Exe/MainOptRobust.m). For both of this actions is necessary to set the value of the parameters in /matlab/Exe/AllRuntimeParameters.m


For both MainExec.m and MainOptRobust.m


- *time_struct*  :  is a struct that defines the duration of the current experiment and the step size = time_struct.ti ; time_struct.tf; time_struct.step,
- *time_sym_struct*  :  the same as above but is used only for inside the simulator,
- *fixed_step*  : defines the type of integration of the differential equations in the simluator,
- *torque_saturation*  : % set the maximum allowed torque in the simulator (in Nm),
- *name_dat*  : defines the name of the controller that we want to use in the current experiment,
- *name_scenario*  : defines the scenario of the current experiment,
- *qi{1}*  : starting joint positions of the robot,
- *qdi{1}*  : starting joint velocities of the robot,
- *choose_alpha = 'RBF'*  :  defines the structure of the activation policies used in this the current experiment. 'RBF' is the default selction,
- *number_of_basis*  : defines basis functions for the RBF,
- *redundancy*  : defines the overlapping of the basis functions,
- *numeric_theta* : set of values that are assigned to the weigths of the RBF. In this way we define the time profile of the wight functions. Not used in      		  	    MainOptRobust.m.

Only for MainOptRobust.m

- *explorationRate*  : defines the exporation rate of CMA-ES
- *niter*  : defines the number of generations of the evolutionary optimization algorithm 
- *fitness*  : defines the fitness that is used to evaluate the quality of the current rollout.
      





