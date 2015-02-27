function s=Linear(tf)

a = 1; 
t = sym('t');

s = pi*(a*t)/(a*tf); % 2*pi is for lemniscate trajectory to perform a complete robit

end