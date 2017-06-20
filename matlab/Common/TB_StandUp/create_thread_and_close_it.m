
% open matlab in a thread and save pid of the sh process that generate it
% (i need to add -r to execute the script to launch )
system('gnome-terminal -x sh -c " cd /home/vale &&  ./save_pid.sh && cd ~/MATLAB/R2013a/bin/ && ./matlab -nodesktop ; bash"')
% i need a pause here to allow the command before to update the file
pause(2);
% open pid file and kill the process
fileID = fopen('pid.txt');
C = textscan(fileID,'%s');
fclose(fileID);
commandkill = strcat('kill -9',{' '},C{1});
pause(5);
system(commandkill{1});
