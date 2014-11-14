%function [x y dx dy ddx ddy] = recordTrajectory()

clear;

figure(1);

h = imfreehand('Closed', false);

% get the position (x,y coordinates) of each point of the curve
positions = getPosition(h);

% translate the curve in order to put its origin in (0,0)
translation = [0 0] - positions(1,:);
newPositions = positions + repmat(translation,size(positions,1),1);
x = newPositions(:,1);
y = newPositions(:,2);

% % plot positions
% figure;
% plot(x,y,'.');

% calculate velocities
dx = x(2:end) - x(1:end-1);
dy = y(2:end) - y(1:end-1);

% calculate accelerations
ddx = (dx(2:end) - dx(1:end-1));
ddy = (dy(2:end) - dy(1:end-1));

% cut down parts where we
% don't have acceleration
dx = dx(2:end);
dy = dy(2:end);
x  = x(3:end);
y  = y(3:end);

% % plot x coordinates, velocities and accelerations with respect to x
% figure;
% hold on;
% 
% plot(x, 'b');
% plot(dx, 'm'); 
% plot(ddx, 'r');
% 
% legend('x', 'dx', 'ddx');

% % plot y coordinates, velocities and accelerations with respect to y
% figure;
% 
% hold on;
% 
% plot(y, 'b');
% plot(dy, 'm'); 
% plot(ddy, 'r');
% 
% legend('y', 'dy', 'ddy');


% define data to be saved
data = [x y dx dy ddx ddy];

save('./stickman1.mat', 'data');

%end
