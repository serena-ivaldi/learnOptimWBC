function plot(obj,chi,param)



ndof  = obj.ndof;


for ii=1:1 %for now we have just one view
    
    %         param.plot_main(ii) = subplot('Position', plot_pos(ii,:));
    %         param.plot_objs{ii} = plot3(0,0,0,'.');
    %         axis([-1 1 -1 1 -1 1]);
    %         axis equal
    %         hold on
    
    param.plot_objs{ii} = plot3(0,0,0,'.');
    axis([-1 1 -1 1 0 1]);
    axis equal
    hold on
    
    %            if param.feet_on_ground(2) == 0 || sum(param.feet_on_ground) == 2
    %
    %               patch([-0.45 -0.45 0.45 0.45],[-0.53 0.37 0.37 -0.53],[0 0 0 0],[0.6 0.6 0.8]);
    %
    %             else
    %
    %               patch([-0.45 -0.45 0.45 0.45],[-0.37 0.53 0.53 -0.37],[0 0 0 0],[0.6 0.6 0.8]);
    %
    %             end
    
    campos([10.3675    4.9702    4.4582]);
    set(gca,'CameraViewAngle',7.8687);
    set(gca,'Color',[0.8 0.8 0.8]);
    set(gca,'XColor',[0.8 0.8 0.8]);
    set(gca,'YColor',[0.8 0.8 0.8]);
    set(gca,'ZColor',[0.8 0.8 0.8]);
    set(gca,'ydir','reverse')
    set(gca,'xdir','reverse')
    set(gca, 'drawmode', 'fast');
    set(gcf, 'Position', get(0, 'Screensize'));
    param.draw_init = 1;
    rotate3d(gca,'on');
    
    %     figure(figure_main);
    
    
    
    %axes(param.plot_main(1))
    %axis tight
    % CoM trajectory
    x_b   = chi(:,1:3);
    qt_b  = chi(:,4:7);
    qj    = chi(:,8:8 + obj.ndof - 1);
    
    visualizeForwardDynamics(obj,[x_b,qt_b,qj],param)
    
    
end

end



function visualizeForwardDynamics(obj,q,params)
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
n       = size(q,1);   % number of instances of the simulation results
qb      = q(:,1:7);    % first 3 elements provide the position and next 4 elements provide the orientation of the base
qj      = q(:,8:8 + obj.ndof - 1 );   % joint positions

vis_speed = 1;         % this variable is set to change the visualization speed,
% to make its speed close to the real time in case
% the simulation time step is changed.


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

kin = zeros(size(q,1),7,n_joint);

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

% constant multipliers related to the sizes of the patches around the links to form the robot figure
% mult_patch = [0.07 , 0.03;
%               0.04 , 0.02;
%               0.03 , 0.02;
%               0.025, 0.02;
%               0.04 , 0.02;
%               0.03 , 0.02;
%               0.025, 0.02;
%               0.03 , 0.02;
%               0.025, 0.02;
%               0.02 , 0.02;
%               0.03 , 0.02;
%               0.025, 0.02;
%               0.02 , 0.02];

%TO DO adapt approx. the size of the boxes to iCub dimension
mult_patch = ones(n_lin,2);
mult_patch(:,1) = mult_patch(:,1)*0.04;
mult_patch(:,2) = mult_patch(:,2)*0.03;

% plot the lines depicting the links
for jj=1:n_lin
    
    lin(jj) = line(xaxis,xyzpairs(jj,1:2),yaxis,xyzpairs(jj,3:4),zaxis,xyzpairs(jj,5:6),'erasemode','normal','linewidth',2,'color','blue');
    
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
    
    lnkpatch(jj) = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);
    
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


lnkpatch(jj)      = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);

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

lnkpatch(jj)      = patch('vertices',xyzpatch.vertices,'faces',xyzpatch.faces,'FaceAlpha',0.2);



% store axes objects' handles to a vector
params.plot_objs{1} = [lnkpatch';lin';x_b0'];
%% UPDATING THE PLOTS
ii=2;

while ii<n+1   % the visualization instance
    
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
    
    ii=ii+vis_speed;
end

end







