## ICUB STANDUP DEMO: INSTRUCTIONS

### Dependencies

In order to run the simulation of iCub standing up from a chair, make sure you have installed on your pc the following dependencies:

 - [yarp](https://github.com/robotology/yarp)
 - [icub-main](https://github.com/robotology/icub-main)
 - [Gazebo simulator](http://gazebosim.org/)
 - [codyco-superbuild](https://github.com/robotology/codyco-superbuild) 

 - Both iCub and the chair models are stored into [icub-gazebo-wholebody](https://github.com/robotology-playground/icub-gazebo-wholebody) repository.
 - [WBToolbox](https://github.com/robotology/WB-Toolbox) and [WBIToolbox-controllers](https://github.com/robotology-playground/WBI-Toolbox-controllers) are required for setting up the simulation.

 It is suggested to install the above mentioned repositories using codyco-superbuild (enable CODYCO_USES_GAZEBO, CODYCO_USES_MATLAB, CODYCO_USES_WBI_TOOLBOX_CONTROLLERS options).
 
### Installation

1) In the `WBIToolbox-controllers` repository, checkout the branch `icub_standup`.

2) In your `.bashrc` file, add the following line:
    
  `alias gazebo_standup="cd ~/YOUR/PATH/TO/icub_standup_world && gazebo -slibgazebo_yarp_clock.so"`
 
   just to initialize the Gazebo world containing iCub on a chair. Instead of opening Gazebo as usual, on a terminal type:

   `gazebo_standup`

   and it will load the correct model.

### Simulations

- To run iCub stand up demo, just use torqueBalancing controller as for other simulations. (for further details see [README](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/master/controllers/torqueBalancing/README.md)). 
  Do not open the yarpmotorgui: iCub is already in its home position!

- In the [initialization file](https://github.com/robotology-playground/WBI-Toolbox-controllers/blob/icub_chair/controllers/torqueBalancing/initTorqueBalancing.m), there is now the new state-machine option "STANDUP" as long as the old ones "YOGA" and "COORDINATOR".

- All the demos "STANDUP", "YOGA" and "COORDINATOR" should be still available without conflicts (BUT with the right Gazebo environment for each demo). 
