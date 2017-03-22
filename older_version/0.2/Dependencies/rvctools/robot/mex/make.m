fprintf('** building mex file\n');

pth = which('find_mex_robotics_toolbox.m');
pth = fileparts(pth);
cd(pth);

mex frne.c ne.c vmath.c
