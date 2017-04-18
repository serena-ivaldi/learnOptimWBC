Follow the instructions to install the learnOptimWBC toolbox.

1) Unzip /learnOptimWBC/matlab/Dependecies/robot-9.10.zip somewhere handy (or in place). This operation will create a folder called rvctools, containing the robotics toolbox that we rely upon. Add /rcvtools and its subfolder to the Matlab PATH (you can right-click on the folder name in the Matlab panel list). To install the robotics toolbox run the rvctools/startup_rvc.m (Every time you have to run a script from the Matlab shell you have to change the current Matlab folder using the directory view that is usually on the right side of the Matlab window) from the Matlab shell. 

1.1) You need to compile the mex file for the robotics toolbox. Depending on your Matlab version, you will need an appropriate compiler version.

For example: Matlab 2013a supports gcc until version 4.4. If you have a newer gcc (probably 4.8 if you have Ubuntu 14.04) then add it to your system:

sudo apt-get install gcc-4.4
sudo apt-get install g++-4.4

Then manually edit the file:
/home/<YOUR_NAME>/.matlab/R2013a/mexopts.sh

and update the name of your compiler:
CC='gcc-4.4'
CXX='g++-4.4'

More instructions on how to set up your compiler: 
http://fr.mathworks.com/help/matlab/matlab_external/changing-default-compiler.html


1.2) compile the mex file for the Newton-Eulero dynamic computation in the robotic toolbox (/rvctools/robot/mex). Check that you added rcvtools to the Matlab PATH (including subfolders). Then run make.m from /rcvtools/robot.

Note: Matlab may complain about line 304 in frne.c and lines 465-468 in ne.c, manually edit the files to change all the \\ comments into /* ... */ comments.

If everything works well, you will read:
>> make
** building mex file


1.3) Move the MEX file frne.xxx (where xxx is the extension of the mex files on your platform - for example frne.mexa64) in rvctools/robot/@SerialLink. The MEX file will now be used instead of the M-file, and thus anything that calls rne() will use the MEX file and be faster. This applies to inertia(), coriolis(), gravload(), and fdyn().


2) once the installation of the robotics toolbox is done go to /learnOptimWBC/matlab and run startup_LOWBC.m. This script will add the paths to Matlab and it will update some files inside the robotics toolbox copying them from the folder /ChangedFile. After this, please remove from the PATH the folder /ChangedFile.

Now you are ready to try learnOptimWBC toolbox on your machine.

3) first of all you need to run the script MainRobotGen.m in learnOptimWBC/matlab/Robot to create a symbolic model (dynamic and kinematic or only kinematic) of the robot described in the file Mdl***.m. Currently this package works only with symbolic model. In the first step you have to provide a kinematic and dynamic description of the robot that you want to use. In the robot folder there are some examples of robots description that have been used by the authors of the package. For the kinematic description we employ the Denavit-Hartenberg convention and the dynamic description needs at least the masses of the links and their inertia matrix. We decide to entirely rely on the @SerialLink class of the robotics toolbox for the description of the robot's dynamic and kinematics (for more information on the @SerialLink class give a look to the robotics toolbox manual). Once you have done with the model, rename the script as Mdl*** where *** is the name of the robot that you want to simulate and save it in /learnOptimWBC/matlab/Robot. Now you can run the MainRobotGen.m script. Change the  line  number 8 with the name of the script that contains the robot model and specify if you want a fully symbolic model (kinematic + dynamic) or just a symbolic kinematic model by changing the value of the flag "genkinonly". Of course a full symbolic models are faster during the integration of the dynamic but it will take longer to compute them. 

4) Now there are several things that you can do with this toolbox:
	
	-you can run a single experiment given a specific scenario, a controller and set of parameters for the activation policies using  /learnOptimWBC/matlab/Exe/MainExec.m,
	-you can compute an optimal parameters set for the activation policies given a scenario and a controller with /learnOptimWBC/matlab/Exe/MainOptRobust.m .

### Possible issues (by Oriane)

* You need to add the toolbox folder to matlab's path.
To add it, click on the "Set Path" icon in the matlab gui, or right-click on the folder in the tree-viewer to add all the learnOptimWBC folder

* In Linux, you may have a problem during the "make" with your gcc compiler.
First you need to know which version of gcc you have:

``gcc -v``

Find the "mexopts.sh" file (launch "find -name "mexopts.sh" from the /usr/local path). 
It should be at the path "/usr/local/MATLAB/R2013a/bin/mexopts.sh" (don't take the one in the toolbox). Modify these lines:

```
l59 : CC = 'gcc' in CC = 'gcc-4.8'
l66 :             CLIBS="$CLIBS -lstdc++" in CLIBS="$CLIBS -L/usr/local/MATLAB/R2013a/sys/os/glnxa64 -lstdc++"
l74 : in CXX='g++-4.8'
l88 : in FC='gfortran-4.8'
```

* If an error about the path appears while compiling, a possible trick is to open ``make.m`` and delete the line ``"CD [...]'``

* If a problem about the comments lines ``//`` appears (because the matlab compiler doesn't understand the ``"//"`` comment in C), open ``frne.c`` and ``ne.c`` (use ``find -name`` if you don't find them) and then change these comments in ``/* comment */``




