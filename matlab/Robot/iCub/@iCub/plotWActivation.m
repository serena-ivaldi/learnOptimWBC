function plotWActivation(obj,chi,params, alphas,names_of_subplot,grouping,colors,legends, varargin)
%% plotWActivation
% Create a subplot figure where the successive positions of the iCub
% according to the matrix chi will be displayed along with the evolutiion
% of the activation function function of the choosen tasks
%   Arguments :
%       chi   - matrix of the base + joints positions on one dimension and
%               the time steps on the other dimension
%      params - Structure of all needed parameters
%      alphas - The activation functions
% names_of_subplot - Cell of strings of the titles of the activation functions 
%               subplots you want to create. By the lenght of this cell you
%               determine the number of subplots created
%    grouping - Cell of vector of the numbers of the alphas you want
%               to plot in the order the want to plot and group them
%      colors - Cell of strings of the Color option you want to use for
%               each line (each alpha). Use 'default' if do not you want to
%               specify a color for a line
%     legends - Cell of strings of the names you want to use as legend in
%               each subplot of activation functions. Use 'none' if you do
%               not want any legend for a line
%     'movie' - Option allowing to save each figure in order to make a
%               video
%  'slowmode' - Activate the slowmode where you have to click on the figure
%               in order to display the next time step
%          fc - If in slowmode you also passe the contact forces vector fc
%               it will also diplay on an other figure the support convex
%               hull and the center of pressures
%
% Example of use 
%       names_of_subplot = {'Right arm tasks','Posture task','Left arm tasks'};
%       grouping = {[1, 2], [5], [3, 4]};
%       colors = {{'r', 'b'}, {'default'}, {'r', 'b'}};
%       legends = {{'elbow','hand'}, {'none'} ,{'elbow','hand'}};
%       icub.plotWActivation(q,input{2},alphas,names_of_subplot,grouping,colors,legends);
%
%       This will plot the iCub along with 3 activation functions plot
%       where the 1st and 2nd alphas are plot in red and blue legended as
%       'elbow' & 'hand' in the first subplot then 5th alpha alone with no
%       color specification nor legend and then the 3rd and 4th alphas together

params.movie = false;
params.slowmode = false;
params.moviepath = [];
ndof  = obj.ndof;
% i check if any option are set for the current plot
if ~(isempty(varargin))
    if strcmp(varargin(1),'movie')
        params.movie = true;
        params.moviepath = varargin(2);
    end
    if strcmp(varargin(1),'slowmode')
        params.slowmode = true;
    end
    if length(varargin) == 2
        params.fc = varargin(2);
    else
        params.fc = [];
    end
end




%configuration of the subplots
wd = length(names_of_subplot); lght = 9;
scene = gca; %copy the current fig into the subplot
axes(wd+1) = subplot(lght,wd,(2*wd+1:wd*lght),scene); %copy the current fig into the subplot
for i = 1:wd
    axes(i) = subplot(lght,wd,[i, i+wd]);
end


for i = 1:wd
    subplot(axes(i))
    hold on
    title(names_of_subplot(i),'FontSize',16)
    xlabel('Time','FontSize',14)
    ylabel('Weight','FontSize',14)
    %axis equal
    axis([0 params.tEnd 0 1]);
    hold off
end

subplot(axes(end))
hold on
axis off

params.plot_objs{1} = plot3(0,0,0,'.');
%axis([-1.2 1.2 -1 1 0 1.2]);
%axis equal


if params.feet_on_ground(2) == 0 || sum(params.feet_on_ground) == 2    
    patch([-0.45 -0.45 0.45 0.45],[-0.53 0.37 0.37 -0.53],[0 0 0 0],[0.6 0.6 0.8]);    
else    
    patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);    
end

%campos([5.1705   -8.3894    6.4718])%([10.3675    4.9702    4.4582]);
%set(gca,'CameraViewAngle',7.8687);
set(gca,'Color',[0.8 0.8 0.8]);
set(gca,'XColor',[0.8 0.8 0.8]);
set(gca,'YColor',[0.8 0.8 0.8]);
set(gca,'ZColor',[0.8 0.8 0.8]);
set(gca,'ydir','reverse')
set(gca,'xdir','reverse')
%set(gca, 'drawmode', 'fast');
set(gcf, 'Position', get(0, 'Screensize'));
rotate3d(gca,'on');
hold off

% CoM trajectory
% x_b   = chi(:,1:3);
% qt_b  = chi(:,4:7);
% qj    = chi(:,8:8 + obj.ndof - 1);

% visualizeForwardDynamics(obj,[x_b,qt_b,qj],params,alphas,names_of_subplot,grouping,legends,axes)
% end
% 
% 
% 
% function visualizeForwardDynamics(obj,chi,params,alphas,names_of_subplot,grouping,legends,axes)
%% visualize_forwardDyn
%   Visualize the simulation results obtained from integration
%   of the forward dynamics of the iCub.
%
%   visualize_forwardDyn(XOUT,PARAMETERS) visualizes the motion
%   of the robot. XOUT is the output vector of the integration carried
%   out in the forward dynamics part, containing the position and the
%   orientation of the base and the joint positions along a time span.
%   PARAMETERS is the struct variable which contains constant parameters
%   related to the simulation environment, robot, controller etc.
%
%% Setup visualization
n       = size(chi,1);   % number of instances of the simulation results
qb      = chi(:,1:7);    % first 3 elements provide the position and next 4 elements provide the orientation of the base
qj      = chi(:,8:8 + obj.ndof - 1 );   % joint positions

vis_speed = 1;         % this variable is set to change the visualization speed,
% to make its speed close to the real time in case
% the simulation time step is changed.
alpha = 0.3; %patches transparency

n_joint = length(obj.jointList); % number of points to be plotted (virtual joints)
n_lin = length(obj.linkList); % number of lines to be plotted (virtual links)


% build the adjacency matrix given the name of the links and the
% information stored in the joints structure (we keep also the info if the
% joint is revolute = 1, prismatic joint = 2; fixed = 3 and 0 is equal no connection);
A = zeros(n_joint,n_joint);

for i = 1:n_joint
    search_string = obj.linkList{i};
    for j = 1:length(obj.jointList)
        if(strcmp(obj.jointList{j}.child.Attributes.link,search_string))
            connection_string = obj.jointList{j}.parent.Attributes.link;
            type_of_connection = obj.jointList{j}.Attributes.type;
            index = find(SubStrFind(connection_string, obj.linkList),1);
            if(strcmp(type_of_connection,'revolute'))
                A(i,index) = 1;
            elseif(strcmp(type_of_connection,'prismatic'))
                A(i,index) = 2;
            elseif(strcmp(type_of_connection,'fixed'))
                A(i,index) = 3;
            end
        end
    end
end

% visualization axis
xaxis = 'xdata';
yaxis = 'ydata';
zaxis = 'zdata';

kin = zeros(size(chi,1),7,n_joint);

for jj=1:n_joint
    for ii=1:n % at each instance
        
        % convert base state to rotation
        [x_b,R_b]    = frame2posrot(squeeze(qb(ii,:)'));
        
        %wbm_setWorldFrame(R_b,x_b,[0,0,-9.81]'); %is this needed ??????
        kin(ii,:,jj)   = (wbm_forwardKinematics(R_b,x_b,qj(ii,:)',obj.linkList{jj}))';  % forward kinematics for the list of joints/links
    end
end

kin(:,:,1) = qb; % use base data instead of fwdKin rootlink

% clear and reset the plots
% axes(params.plot_main(ii));
% cla;
% axis off;

%     if params.feet_on_ground(2) == 0 || sum(params.feet_on_ground) == 2
%
%     patch([-0.45 -0.45 0.45 0.45],[-0.53 0.37 0.37 -0.53],[0 0 0 0],[201 220 222]./255);
%
%     else
%
%     patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[201 220 222]./255);
%
%     end

drawnow

%axes(params.plot_main(1));

%% INITIAL PLOTS
subplot(axes(end))
hold on
% allocate memory
x        = zeros(1,n_joint);
y        = zeros(1,n_joint);
z        = zeros(1,n_joint);
x_b0     = zeros(1,n_joint);
xyzpairs = zeros(n_lin,6);

% plot the base position
x(1) = kin(1,1,1);
y(1) = kin(1,2,1);
z(1) = kin(1,3,1);

x_b0(1) = plot3(x(1),y(1),z(1),'r*');

% plot the joints

for jj = 2:n_joint-1
    
    [x_btemp,~] = frame2posrot(kin(1,:,jj)');
    x(jj)       = x_btemp(1);
    y(jj)       = x_btemp(2);
    z(jj)       = x_btemp(3);
    
    col         = 'r.';
    x_b0(jj)    = plot3(x(jj),y(jj),z(jj),col,'MarkerSize', 25);
    
end

% plot the position of the COM
jj = n_joint;

[x_btemp,~] = frame2posrot(kin(1,:,jj)');
x(jj)   = x_btemp(1);
y(jj)   = x_btemp(2);
z(jj)   = x_btemp(3);

x_b0(jj)    = plot3(x(jj),y(jj),z(jj),'g*');

%% LINKS DEFINITION
% define the pairs between the joints that will form the links
% i build the link connection using the info from the connectivity matrix A
for i = 1:n_joint
    row = zeros(1,6);
    logical_row = A(i,:)>=1;
    [result,index]=find(logical_row,1);
    if(result)
        xyzpairs(i,:) = [x(index)  x(i)  y(index)  y(i)  z(index)  z(i)];
    else
        xyzpairs(i,:) = row;
    end
end

% allocate memory
lin               = zeros(1,n_joint);
lnkpatch          = zeros(1,n_joint);
xyzpatch.vertices = zeros(8,3);
xyzpatch.faces    = zeros(6,4);

%TO DO adapt approx. the size of the boxes to iCub dimension
mult_patch = ones(n_lin,2);
mult_patch(:,1) = mult_patch(:,1)*0.03;
mult_patch(:,2) = mult_patch(:,2)*0.03;

try
    %legs
    mult_patch(strcmp(obj.linkList,'r_lower_leg'),:) = [0.035 0.03];
    mult_patch(strcmp(obj.linkList,'r_ankle_1'),:) = [0.025 0.03];
    mult_patch(strcmp(obj.linkList,'l_lower_leg'),:) = [0.035 0.03];
    mult_patch(strcmp(obj.linkList,'l_ankle_1'),:) = [0.025 0.03];
    %remove dh_frame and skin frames
    mult_patch(strcmp(obj.linkList,'r_hand_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'l_forearm_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'r_forearm_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'l_hand_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'r_foot_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'l_foot_dh_frame'),:) = [0 0];
    mult_patch(strcmp(obj.linkList,'chest_skin_frame'),:) = [0 0];
    %torso
    mult_patch(strcmp(obj.linkList,'r_hip_1'),:) = [0.02 0.02];
    mult_patch(strcmp(obj.linkList,'l_hip_1'),:) = [0.02 0.02];
    mult_patch(strcmp(obj.linkList,'r_shoulder_1'),:) = [0.02 0.02];
    mult_patch(strcmp(obj.linkList,'l_shoulder_1'),:) = [0.02 0.02];
    %arms
    mult_patch(strcmp(obj.linkList,'r_gripper'),:) = [0.01 0.035];
    mult_patch(strcmp(obj.linkList,'r_hand'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'r_forearm'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'r_shoulder_2'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'r_shoulder_3'),:) = [0.01 0.01];
    
    mult_patch(strcmp(obj.linkList,'r_shoulder_3'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'l_shoulder_2'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'l_gripper'),:) = [0.01 0.035];
    mult_patch(strcmp(obj.linkList,'l_hand'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'l_forearm'),:) = [0.01 0.01];
    %head
    mult_patch(strcmp(obj.linkList,'imu_frame'),:) = [0.05 0.05];
    mult_patch(strcmp(obj.linkList,'head'),:) = [0.01 0.01];
    mult_patch(strcmp(obj.linkList,'neck_2'),:) = [0.02 0.02];
catch err
    disp('The URDF file seems to disrespect name convention or some links are missing in the URDF');
end


% plot the lines depicting the links
for jj=1:n_lin
    
    lin(jj) = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',2,'color','b');% [0 191 255]./255);
    
    % for the patches (to determine the orientation of the patch to be applied to the links)
    vectlnk  = [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
    orthlnk  = null(vectlnk);
    orthlnk1 = mult_patch(jj,1)*orthlnk(:,1);
    orthlnk2 = mult_patch(jj,2)*orthlnk(:,2);
    
    % offsets in the direction orthogonal to the link
    qq1      =  orthlnk1+orthlnk2;
    qq2      = -orthlnk1+orthlnk2;
    qq3      = -orthlnk1-orthlnk2;
    qq4      =  orthlnk1-orthlnk2;
    
    % vertices for the patch
    xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
        xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
        xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
        xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);
        xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
        xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
        xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
        xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
    
    
    xyzpatch.faces   = [ 1 2 3 4;
        1 4 8 5;
        5 8 7 6;
        7 3 2 6;
        2 6 5 1;
        3 7 8 4];
    
    lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',alpha);
    
    
end

% right foot patch
jj=n_lin+1;

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 =  orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 =  orthlnk1-orthlnk2;

r_foot_index = find(strcmp(obj.linkList,'r_foot'));

xyzpatch.vertices = [xyzpairs(r_foot_index,2)+qq1(1)      , xyzpairs(r_foot_index,4)+qq1(2) , xyzpairs(r_foot_index,6)+qq1(3);
    xyzpairs(r_foot_index,2)+qq2(1)      , xyzpairs(r_foot_index,4)+qq2(2) , xyzpairs(r_foot_index,6)+qq2(3);
    xyzpairs(r_foot_index,2)+qq3(1)      , xyzpairs(r_foot_index,4)+qq3(2) , xyzpairs(r_foot_index,6)+qq3(3);
    xyzpairs(r_foot_index,2)+qq4(1)      , xyzpairs(r_foot_index,4)+qq4(2) , xyzpairs(r_foot_index,6)+qq4(3);
    xyzpairs(r_foot_index,2)+qq1(1)+0.03 , xyzpairs(r_foot_index,4)+qq1(2) , xyzpairs(r_foot_index,6)+qq1(3);
    xyzpairs(r_foot_index,2)+qq2(1)+0.03 , xyzpairs(r_foot_index,4)+qq2(2) , xyzpairs(r_foot_index,6)+qq2(3);
    xyzpairs(r_foot_index,2)+qq3(1)+0.03 , xyzpairs(r_foot_index,4)+qq3(2) , xyzpairs(r_foot_index,6)+qq3(3);
    xyzpairs(r_foot_index,2)+qq4(1)+0.03 , xyzpairs(r_foot_index,4)+qq4(2) , xyzpairs(r_foot_index,6)+qq4(3)];


lnkpatch(jj)      = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',alpha);

% left foot patch
jj=n_lin+2;

orthlnk1 = [0 0.03 0]';
orthlnk2 = [0 0 0.03]';

qq1 =  orthlnk1+2*orthlnk2;
qq2 = -orthlnk1+2*orthlnk2;
qq3 = -orthlnk1-orthlnk2;
qq4 =  orthlnk1-orthlnk2;

l_foot_index = find(strcmp(obj.linkList,'l_foot'));
xyzpatch.vertices = [xyzpairs(l_foot_index,2)+qq1(1)      , xyzpairs(l_foot_index,4)+qq1(2) , xyzpairs(l_foot_index,6)+qq1(3);
    xyzpairs(l_foot_index,2)+qq2(1)      , xyzpairs(l_foot_index,4)+qq2(2) , xyzpairs(l_foot_index,6)+qq2(3);
    xyzpairs(l_foot_index,2)+qq3(1)      , xyzpairs(l_foot_index,4)+qq3(2) , xyzpairs(l_foot_index,6)+qq3(3);
    xyzpairs(l_foot_index,2)+qq4(1)      , xyzpairs(l_foot_index,4)+qq4(2) , xyzpairs(l_foot_index,6)+qq4(3);
    xyzpairs(l_foot_index,2)+qq1(1)+0.03 , xyzpairs(l_foot_index,4)+qq1(2) , xyzpairs(l_foot_index,6)+qq1(3);
    xyzpairs(l_foot_index,2)+qq2(1)+0.03 , xyzpairs(l_foot_index,4)+qq2(2) , xyzpairs(l_foot_index,6)+qq2(3);
    xyzpairs(l_foot_index,2)+qq3(1)+0.03 , xyzpairs(l_foot_index,4)+qq3(2) , xyzpairs(l_foot_index,6)+qq3(3);
    xyzpairs(l_foot_index,2)+qq4(1)+0.03 , xyzpairs(l_foot_index,4)+qq4(2) , xyzpairs(l_foot_index,6)+qq4(3)];

lnkpatch(jj)      = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',alpha);



% store axes objects' handles to a vector
params.plot_objs{1} = [lnkpatch';lin';x_b0'];
hold off

%% init plots activation functions
for kk = 1:wd
    subplot(axes(kk))
    hold on
    for idx = 1:length(grouping{kk})
        alphas{1,grouping{kk}(1,idx)}.Plot;
    end
    handle = get(axes(kk), 'Children');
    for idx = 1:length(handle)
        handle(idx).LineWidth = 2;
        if ~strcmp(colors{1,kk}(1,idx), 'default')
            color = colors{1,kk}{1,idx};
            handle(idx).Color = color;
        end        
    end
    if ~(strcmp(legends{1,kk},'none'))
        leg = legend(legends{1,kk});
        leg.FontSize = 15;
        set(leg,'Location','southeast');
    end
    t = 1/n*params.tEnd;
    tline(kk) = line([t t],[0 1],'LineWidth',2,'Color','k');
    hold off
end

%% UPDATING THE PLOTS
ii=2;
movieframenum = 1;

fig = gcf;
fig.PaperPositionMode = 'auto'; %usefull for the movie option

while ii<n+1   % the visualization instance
    subplot(axes(end))
    hold on
    
    tic;      % visualizer step timer start (to setting the visualizer speed)
    
    % get the positions for the current instance
    for jj=1:n_joint
        [x_btemp,~] = frame2posrot(kin(ii,:,jj)');
        x(jj) = x_btemp(1);
        y(jj) = x_btemp(2);
        z(jj) = x_btemp(3);
        set(x_b0(jj),xaxis,x(jj),yaxis,y(jj),zaxis,z(jj));
    end
    
    %	update the joints positions
    for i = 1:n_joint
        row = zeros(1,6);
        logical_row = A(i,:)>=1;
        [result,index]=find(logical_row,1);
        if(result)
            xyzpairs(i,:) = [x(index)  x(i)  y(index)  y(i)  z(index)  z(i)];
        else
            xyzpairs(i,:) = row;
        end
    end
    
    % update the lines for the links wrt the new joint positions
    for jj=1:n_lin
        
        set(lin(jj),xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6));
        
        vectlnk   =  [xyzpairs(jj,2)-xyzpairs(jj,1),xyzpairs(jj,4)-xyzpairs(jj,3),xyzpairs(jj,6)-xyzpairs(jj,5)];
        orthlnk   =  null(vectlnk);
        orthlnk1  =  mult_patch(jj,1)*orthlnk(:,1);
        orthlnk2  =  mult_patch(jj,2)*orthlnk(:,2);
        qq1       =  orthlnk1+orthlnk2;
        qq2       = -orthlnk1+orthlnk2;
        qq3       = -orthlnk1-orthlnk2;
        qq4       =  orthlnk1-orthlnk2;
        
        xyzpatch.vertices = [xyzpairs(jj,2)+qq1(1) , xyzpairs(jj,4)+qq1(2) , xyzpairs(jj,6)+qq1(3);
            xyzpairs(jj,2)+qq2(1) , xyzpairs(jj,4)+qq2(2) , xyzpairs(jj,6)+qq2(3);
            xyzpairs(jj,2)+qq3(1) , xyzpairs(jj,4)+qq3(2) , xyzpairs(jj,6)+qq3(3);
            xyzpairs(jj,2)+qq4(1) , xyzpairs(jj,4)+qq4(2) , xyzpairs(jj,6)+qq4(3);
            xyzpairs(jj,1)+qq1(1) , xyzpairs(jj,3)+qq1(2) , xyzpairs(jj,5)+qq1(3);
            xyzpairs(jj,1)+qq2(1) , xyzpairs(jj,3)+qq2(2) , xyzpairs(jj,5)+qq2(3);
            xyzpairs(jj,1)+qq3(1) , xyzpairs(jj,3)+qq3(2) , xyzpairs(jj,5)+qq3(3);
            xyzpairs(jj,1)+qq4(1) , xyzpairs(jj,3)+qq4(2) , xyzpairs(jj,5)+qq4(3)];
        
        set(lnkpatch(jj),'vertices',xyzpatch.vertices);
        
    end
    
    % feet patches
    % right foot
    jj=n_lin+1;
    
    orthlnk1 = [0 0.03 0]';
    orthlnk2 = [0 0 0.03]';
    
    qq1 =  orthlnk1+2*orthlnk2;
    qq2 = -orthlnk1+2*orthlnk2;
    qq3 = -orthlnk1-orthlnk2;
    qq4 =  orthlnk1-orthlnk2;
    
    xyzpatch.vertices = [xyzpairs(r_foot_index,2)+qq1(1)      , xyzpairs(r_foot_index,4)+qq1(2) , xyzpairs(r_foot_index,6)+qq1(3);
        xyzpairs(r_foot_index,2)+qq2(1)      , xyzpairs(r_foot_index,4)+qq2(2) , xyzpairs(r_foot_index,6)+qq2(3);
        xyzpairs(r_foot_index,2)+qq3(1)      , xyzpairs(r_foot_index,4)+qq3(2) , xyzpairs(r_foot_index,6)+qq3(3);
        xyzpairs(r_foot_index,2)+qq4(1)      , xyzpairs(r_foot_index,4)+qq4(2) , xyzpairs(r_foot_index,6)+qq4(3);
        xyzpairs(r_foot_index,2)+qq1(1)+0.03 , xyzpairs(r_foot_index,4)+qq1(2) , xyzpairs(r_foot_index,6)+qq1(3);
        xyzpairs(r_foot_index,2)+qq2(1)+0.03 , xyzpairs(r_foot_index,4)+qq2(2) , xyzpairs(r_foot_index,6)+qq2(3);
        xyzpairs(r_foot_index,2)+qq3(1)+0.03 , xyzpairs(r_foot_index,4)+qq3(2) , xyzpairs(r_foot_index,6)+qq3(3);
        xyzpairs(r_foot_index,2)+qq4(1)+0.03 , xyzpairs(r_foot_index,4)+qq4(2) , xyzpairs(r_foot_index,6)+qq4(3)];
    
    set(lnkpatch(jj),'vertices',xyzpatch.vertices);
    
    % left foot
    jj=n_lin+2;
    
    orthlnk1 = [0 0.03 0]';
    orthlnk2 = [0 0 0.03]';
    
    qq1 =  orthlnk1+2*orthlnk2;
    qq2 = -orthlnk1+2*orthlnk2;
    qq3 = -orthlnk1-orthlnk2;
    qq4 =  orthlnk1-orthlnk2;
    
    xyzpatch.vertices = [xyzpairs(l_foot_index,2)+qq1(1)      , xyzpairs(l_foot_index,4)+qq1(2) , xyzpairs(l_foot_index,6)+qq1(3);
        xyzpairs(l_foot_index,2)+qq2(1)      , xyzpairs(l_foot_index,4)+qq2(2) , xyzpairs(l_foot_index,6)+qq2(3);
        xyzpairs(l_foot_index,2)+qq3(1)      , xyzpairs(l_foot_index,4)+qq3(2) , xyzpairs(l_foot_index,6)+qq3(3);
        xyzpairs(l_foot_index,2)+qq4(1)      , xyzpairs(l_foot_index,4)+qq4(2) , xyzpairs(l_foot_index,6)+qq4(3);
        xyzpairs(l_foot_index,2)+qq1(1)+0.03 , xyzpairs(l_foot_index,4)+qq1(2) , xyzpairs(l_foot_index,6)+qq1(3);
        xyzpairs(l_foot_index,2)+qq2(1)+0.03 , xyzpairs(l_foot_index,4)+qq2(2) , xyzpairs(l_foot_index,6)+qq2(3);
        xyzpairs(l_foot_index,2)+qq3(1)+0.03 , xyzpairs(l_foot_index,4)+qq3(2) , xyzpairs(l_foot_index,6)+qq3(3);
        xyzpairs(l_foot_index,2)+qq4(1)+0.03 , xyzpairs(l_foot_index,4)+qq4(2) , xyzpairs(l_foot_index,6)+qq4(3)];
    
    set(lnkpatch(jj),'vertices',xyzpatch.vertices);
    
    % end feet patches
    
    % store axes objects to a vector
    params.plot_objs{1} = [lnkpatch';lin';x_b0'];
    
    
    drawnow;
    
    % to update the visualizer speed to keep it close to real simulation time
    time_dif = vis_speed*params.sim_step-toc();
    
    if time_dif>0
        
        pause(time_dif);
    else
        
        vis_speed=vis_speed+1;
    end
    hold off    
    
    %% Update the activation function plots
    for kk = 1:wd
        subplot(axes(kk))
        hold on
        t = ii/n*params.tEnd;
        tline(kk).delete
        tline(kk) = line([t t],[0 1],'LineWidth',2,'Color','k'); %line to display the time on the activation plots
        hold off
    end
    
    %% Add a frame to the movie
    if params.movie
        % write the frame to the movie folder
        path = fullfile(params.moviepath, sprintf('%04d.png', movieframenum));
        saveas(gcf, path{1});
        movieframenum = movieframenum + 1;
    end
    
    %% Slow mode
    %you need to click the figure to get the next frame
    if params.slowmode
        convHull = obj.computeSupPoly(params.feet_on_ground,chi(ii-1,:)');
        if ii == 2
            figCvHull = figure;
        end
        
        CoP(1)      = -params.fc{1}(5,ii-1)/params.fc{1}(3,ii-1);
        CoP(2)      =  params.fc{1}(4,ii-1)/params.fc{1}(3,ii-1);
        
        if  params.numContacts == 2
            
            CoP(3)      = -params.fc{1}(11,ii-1)/params.fc{1}(9,ii-1);
            CoP(4)      =  params.fc{1}(10,ii-1)/params.fc{1}(9,ii-1);
            
            q_cur = chi(ii-1,:);
            [trans, rot] = obj.offlineFkine(q_cur','r_sole');
            temp = [CoP(3), CoP(4), 0]' + trans;
            temp = rot*temp;
            CoP(3) = temp(1);
            CoP(4) = temp(2);
            
        end
        
        convHull.plotConvHull(figCvHull,CoP(1),CoP(2));
        convHull.plotConvHull(figCvHull,CoP(3),CoP(4));
        waitforbuttonpress
    end
    
    
    ii=ii+vis_speed;
end

end
