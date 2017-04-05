function s=Linear(tf)

a = 1; 
t = sym('t');

s = pi*((a*t)/(a*tf)); % multiply all for 2*pi to have a complete lemniscate trajectory

end