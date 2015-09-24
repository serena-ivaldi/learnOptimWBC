// This file is part of the REMOTE API
// 
// Copyright 2006-2014 Coppelia Robotics GmbH. All rights reserved. 
// marc@coppeliarobotics.com
// www.coppeliarobotics.com
// 
// The REMOTE API is licensed under the terms of GNU GPL:
// 
// -------------------------------------------------------------------
// The REMOTE API is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// THE REMOTE API IS DISTRIBUTED "AS IS", WITHOUT ANY EXPRESS OR IMPLIED
// WARRANTY. THE USER WILL USE IT AT HIS/HER OWN RISK. THE ORIGINAL
// AUTHORS AND COPPELIA ROBOTICS GMBH WILL NOT BE LIABLE FOR DATA LOSS,
// DAMAGES, LOSS OF PROFITS OR ANY OTHER KIND OF LOSS WHILE USING OR
// MISUSING THIS SOFTWARE.
// 
// See the GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with the REMOTE API.  If not, see <http://www.gnu.org/licenses/>.
// -------------------------------------------------------------------
//
// This file was automatically created for V-REP release V3.1.3 on Sept. 30th 2014

/* Your custom remote API functions. Following are examples: */

if (_rawCmdID==simx_customcmd_get_time)
{
	// The data that is part of the command (i.e. the name of the object) is stored in _cmdString

	// Execute the command now:
	simFloat time = simGetSimulationTime();
        //printf("time = %f \n",time); 
	// We now need to return the script handle:
	// We now need to return that number (index):
		retCmd->setDataReply_1float(time,true,otherSideIsBigEndian); // Endianness of the client is detected and automatically adjusted (for simple data replies, otherwise you'll have to do that yourself)

}
if (_rawCmdID==simx_customcmd_get_delta)
{
	// The data that is part of the command (i.e. the name of the object) is stored in _cmdString

	// Execute the command now:
	simFloat delta = simGetSimulationTimeStep();
        //printf("delta = %f \n",delta);
	// We now need to return the script handle:
	// We now need to return that number (index):
    retCmd->setDataReply_1float(delta,true,otherSideIsBigEndian); // Endianness of the client is detected and automatically adjusted (for simple data replies, otherwise you'll have to do that yourself)
}




