function text = LoadScenario(name_scenario)

allpath=which('FindData.m');
path=fileparts(allpath);
filename=strcat(path,'/scenarios/',name_scenario,'.txt');
text = fileread(filename);


end