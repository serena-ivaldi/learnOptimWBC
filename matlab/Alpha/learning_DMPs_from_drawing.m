clear all
close all
clc

%%% Learn DMPs from drawings

% call the drawing script
recordTrajectory;

T = length(x);

tau = T/500;
alpha_z = 0.1;

%% Define the phase variable z as a function of the time t
z = @(t) exp(-tau*alpha_z*t);

Z = zeros(T,1);

for i = 1:T
    Z(i) = z(i);
end

%plot(t,Z);

h = T;

%% Define some basis functions
phi1 = @(t) exp(-0.5*(t-1)^2/h); 
phi2 = @(t) exp(-0.5*(t-T/9)^2/h);
phi3 = @(t) exp(-0.5*(t-2*T/9)^2/h);
phi4 = @(t) exp(-0.5*(t-3*T/9)^2/h);
phi5 = @(t) exp(-0.5*(t-4*T/9)^2/h);
phi6 = @(t) exp(-0.5*(t-5*T/9)^2/h);
phi7 = @(t) exp(-0.5*(t-6*T/9)^2/h);
phi8 = @(t) exp(-0.5*(t-7*T/9)^2/h);
phi9 = @(t) exp(-0.5*(t-8*T/9)^2/h);
phi10 = @(t) exp(-0.5*(t-T)^2/h);

PHI1 = zeros(T,1);
PHI2 = zeros(T,1);
PHI3 = zeros(T,1);
PHI4 = zeros(T,1);
PHI5 = zeros(T,1);
PHI6 = zeros(T,1);
PHI7 = zeros(T,1);
PHI8 = zeros(T,1);
PHI9 = zeros(T,1);
PHI10 = zeros(T,1);


for i = 1:T
    PHI1(i) = phi1(i);
    PHI2(i) = phi2(i);
    PHI3(i) = phi3(i);
    PHI4(i) = phi4(i);
    PHI5(i) = phi5(i);
    PHI6(i) = phi6(i);
    PHI7(i) = phi7(i);
    PHI8(i) = phi8(i);
    PHI9(i) = phi9(i);
    PHI10(i) = phi10(i);
end

figure(2);
hold all;
plot(PHI1);
plot(PHI2);
plot(PHI3);
plot(PHI4);
plot(PHI5);
plot(PHI6);
plot(PHI7);
plot(PHI8);
plot(PHI9);
plot(PHI10);

%% Define PSIs

%SUM_PHIs = sum([PHI1'; PHI2'; PHI3'; PHI4'; PHI5'; PHI6'; PHI7'; PHI8'; PHI9'; PHI10']);

PSI1 = (PHI1.*Z);
PSI2 = (PHI2.*Z);
PSI3 = (PHI3.*Z);
PSI4 = (PHI4.*Z);
PSI5 = (PHI5.*Z);
PSI6 = (PHI6.*Z);
PSI7 = (PHI7.*Z);
PSI8 = (PHI8.*Z);
PSI9 = (PHI9.*Z);
PSI10 = (PHI10.*Z);

figure(3);
hold all;
plot(PSI1);
plot(PSI2);
plot(PSI3);
plot(PSI4);
plot(PSI5);
plot(PSI6);
plot(PSI7);
plot(PSI8);
plot(PSI9);
plot(PSI10);

%% Define our desired trajectory

% initial position
sx0 = x(1);
sy0 = y(1);

% initial velocity
vx0 = dx(1);
vy0 = dy(1);

% desired position
sxd = x(end);
syd = y(end);

% desired velocity
vxd = dx(end);
vyd = dy(end);

% parameters of the PD controller
alpha = 10;
beta = 1;

sxt = sx0;
vxt = vx0;

syt = sy0;
vyt = vy0;

Sx = x; % positions
Vx = dx; % velocities
Ax = ddx; % accelerations

Sy = y; % positions
Vy = dy; % velocities
Ay = ddy; % accelerations

figure(4);
hold all;
plot(Sx);
%plot(t,V);
%plot(t,A);
%legend('positions', 'velocities', 'accelerations');

figure(5);
hold all;
plot(Sy);

%% Compute the forcing function given the desired trajectory
fx = Ax - alpha*beta*(ones(length(Sx),1)*sxd - Sx) + alpha*Vx;
fy = Ay - alpha*beta*(ones(length(Sy),1)*syd - Sy) + alpha*Vy;

figure(10);
hold all;
plot(fx);

%% Find the right weights to define the DMP
wx = ([PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']')^-1*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']*fx;
wy = ([PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']')^-1*[PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']*fy;

%% Define the forcing function using the learned weights

fx = [PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']'*wx;
fy = [PSI1'; PSI2'; PSI3'; PSI4'; PSI5'; PSI6'; PSI7'; PSI8'; PSI9'; PSI10']'*wy;

figure(10);
plot(fx);
legend('ground truth', 'approximation');

%% Generate the trajectory using now the DMP

% initial position
sx0 = x(1);
sy0 = y(1);

% initial velocity
vx0 = dx(1);
vy0 = dy(1);

% desired position
sxd = x(end);
syd = y(end);

% desired velocity
vxd = dx(end);
vyd = dy(end);

% parameters of the PD controller
alpha = 10;
beta = 1;

sxt = sx0;
vxt = vx0;

syt = sy0;
vyt = vy0;

Sx = zeros(T,1); % positions
Vx = zeros(T,1); % velocities
Ax = zeros(T,1); % accelerations

Sy = zeros(T,1); % positions
Vy = zeros(T,1); % velocities
Ay = zeros(T,1); % accelerations

% simulation
for i = 1:T
   
   Sx(i) = sxt;
   Vx(i) = vxt;
   
   Sy(i) = syt;
   Vy(i) = vyt;
    
   ax = alpha*beta*(sxd-sxt) + alpha*(vxd-vxt) + fx(i);
   ay = alpha*beta*(syd-syt) + alpha*(vyd-vyt) + fy(i);
   
   Ax(i) = ax;
   Ay(i) = ay;
    
   %new position
   sxt = sxt + vxt*0.2 + ax*0.2^2/2;
   syt = syt + vyt*0.2 + ay*0.2^2/2;
   
   %new velocity
   vxt = vxt + ax*0.2;
   vyt = vyt + ay*0.2;
    
end

figure(4);
plot(Sx);
legend('original', 'reproduction');

figure(5);
plot(Sy);
legend('original', 'reproduction');

figure(6);
hold all;
plot(x, y);
plot(Sx, Sy);
legend('original', 'reproduction');


