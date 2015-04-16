function s=Linear(tf)

a = 1; 
t = sym('t');

s = (a*t)/(a*tf); % 2*pi is for lemniscate trajectory to perform a complete orbit

end