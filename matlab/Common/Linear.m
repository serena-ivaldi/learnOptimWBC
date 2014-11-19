function s=Linear(tf,time_parameters)

a = time_parameters(1); 
t = sym('t');

s = (a*t)/(a*tf);

end