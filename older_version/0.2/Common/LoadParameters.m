function fullpath = LoadParameters(name)

allpath=which('FindData.m');
path=fileparts(allpath);
fullpath=strcat(path,'/datamat/',name,'.mat');


end
