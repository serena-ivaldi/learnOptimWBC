Follow the instructions to install the learnOptimWBC toolbox.

1) Unzip /learnOptimWBC/matlab/Dependecies/robot-9.10.zip somewhere handy. This operation will create a folder called rvctools. To install the robotics toolbox run the rvctools/startup_rvc.m (Every time you have to run a script from the Matlab shell you have to change the current Matlab folder using the directory view that is usually on the right side of the Matlab window) from the Matlab shell. 

1.1) compile the mex file for the Newton-Eulero dynamic computation in the robotic toolbox (/rvctools/robot/mex). Just run make.m from MATLAB
which assumes that the mex utility is in your current path.  This is typically found in the bin subdirectory of your installed MATLAB. You do of course also
need a C compiler. after that Move the MEX file frne.xxx (where xxx is the extension of the mex files on your platform) in rvctools/robot/@SerialLink. The MEX file will now be used instead of the M-file, and thus anything that calls rne() will use the MEX file and be faster. This applies to inertia(), coriolis(), gravload(), and fdyn().

2) once the installation of the robotics toolbox is done go to /learnOptimWBC/matlab and run startup_LOWBC.m. This script will add the paths to Matlab and it will update some files inside the robotics toolbox. Now you are ready to try learnOptimWBC toolbox on your machine.

3) first of all you need to run the script MainRobotGen.m in learnOptimWBC/matlab/Robot to create a symbolic model (dynamic and kinematic or only kinematic) of the robot described in the file Mdl***.m. Currently this package works only with symbolic model. In the first step you have to provide a kinematic and dynamic description of the robot that you want to use. In the robot folder there are some examples of robots description that have been used by the authors of the package. For the kinematic description we employ the Denavit-Hartenberg convention and the dynamic description needs at least the masses of the links and their inertia matrix. We decide to entirely rely on the @SerialLink class of the robotics toolbox for the description of the robot's dynamic and kinematics (for more information on the @SerialLink class give a look to the robotics toolbox manual). Once you have done with the model, rename the script as Mdl*** where *** is the name of the robot that you want to simulate and save it in /learnOptimWBC/matlab/Robot. Now you can run the MainRobotGen.m script. Change the  line  number 8 with the name of the script that contains the robot model and specify if you want a fully symbolic model (kinematic + dynamic) or just a symbolic kinematic model by changing the value of the flag "genkinonly". Of course a full symbolic models are faster during the integration of the dynamic but it will take longer to compute them. 

4) Now there are several things that you can do with this toolbox:
	
	-you can run a single experiment given a specific scenario, a controller and set of parameters for the activation policies using  /learnOptimWBC/matlab/Exe/MainExec.m,
	-you can compute an optimal parameters set for the activation policies given a scenario and a controller with /learnOptimWBC/matlab/Exe/MainOptRobust.m .



