%SerialLink.animate   Update a robot animation
%
% R.animate(q) updates an existing animation for the robot R.  This will have
% been created using R.plot().
%
% Updates graphical instances of this robot in all figures.
%
% Notes::
% - Called by plot() and plot3d() to actually move the arm models
% - Used for Simulink robot animation.
%
% See also SerialLink.plot.

% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function animate(controller,qq,time)

    if nargin < 4
        handles = findobj('Tag', controller.subchains.name);
    end
    
    links = controller.subchains.links;
    N = controller.subchains.n;
    
    disp(N)
    
    
    % get handle of any existing graphical robots of same name
    %  one may have just been created above
    handles = findobj('Tag', controller.subchains.name);
    
    % index k for downsampling visualization  
    k = 1;
    % inizializes plot handles
    pos = zeros(3,controller.references.GetNumTasks());
    for ii = 1:controller.references.GetNumTasks()
       pos(:,ii) = controller.references.GetTraj(ii,time(1));
       des_traj_pos(ii) = plot3(pos(1,ii),pos(2,ii),pos(3,ii),'-.r*','MarkerSize',10,'XDataSource','pos(1,ii)','YDataSource','pos(2,ii)','ZDataSource','pos(3,ii)');
    end
    % MAIN DISPLAY/ANIMATION LOOP
    while true
        % animate over all instances of this robot in different axes
        
        for i=2:size(time,1)  
            %check if time fixed_step is active and the the time step is sufficiently large 
            if(time(i)-time(k)>controller.display_opt.step)
               q = qq(i,:);
               for handle=handles'
   %                 h = get(handle, 'UserData');
   %                 h.q = q';
   %                 set(handle, 'UserData', h);

                     group = findobj('Tag', controller.subchains.name);
                     h = get(group, 'UserData');

                   % now draw it for a pose q
                   if controller.subchains.mdh
                       % modified DH case
                       T = controller.subchains.base;
                       vert = transl(T)';

                       for L=1:N
                           if controller.subchains.links(L).isprismatic()
                               set(h.pjoint(L), 'Matrix', trotz(q(L))*diag([1 1 -q(L) 1]));
                           end
                           T = T * links(L).A(q(L));
                           set(h.link(L), 'Matrix', T); 
                           vert = [vert; transl(T)'];
                       end
                       % update the transform for link N+1 (the tool)
                       T = T * controller.subchains.tool;
                       if length(h.link) > N
                           set(h.link(N+1), 'Matrix', T);
                       end
                       vert = [vert; transl(T)'];
                   else
                       % standard DH case
                       T = controller.subchains.base;
                       vert = transl(T)';

                       for L=1:N
                           % for all N+1 links
                           if controller.subchains.links(L).isprismatic()
                               set(h.pjoint(L), 'Matrix', T*trotz(q(L))*diag([1 1 q(L) 1]));
                           end
                           if h.link(L) ~= 0
                               set(h.link(L), 'Matrix', T);
                           end
                           
                           T = T * links(L).A(q(L));
                           vert = [vert; transl(T)'];
                       end
                       % update the transform for link N+1 (the tool)
                       if length(h.link) > N
                           set(h.link(N+1), 'Matrix', T);
                       end
                       T = T*controller.subchains.tool;
                       vert = [vert; transl(T)'];
                   end

                   % now draw the shadow
   %                 if ~isempty(robot.tool)
   %                     t = transl(T*robot.tool);
   %                     vl = vert(end,:);
   %                     if t(1) ~= 0
   %                         vert = [vert; [t(1) vl(2) vl(3)]];
   %                     end
   %                     if t(2) ~= 0
   %                         vert = [vert; [t(1) t(2) vl(3)]];
   %                     end
   %                     if t(3) ~= 0
   %                         vert = [vert; t'];
   %                     end
   %                 end
                   if isfield(h, 'shadow')
                       set(h.shadow, 'Xdata', vert(:,1), 'Ydata', vert(:,2), ...
                           'Zdata', h.floorlevel*ones(size(vert(:,1))));
                   end

                   % update the tool tip trail
                   if isfield(h, 'trail')
                       T = controller.subchains.fkine(q);
                       controller.subchains.trail = [controller.subchains.trail; transl(T)'];
                       set(h.trail, 'Xdata', controller.subchains.trail(:,1), 'Ydata', controller.subchains.trail(:,2), 'Zdata', controller.subchains.trail(:,3));
                   end

   %                 T = T * robot.tool;
   %                 vert = [vert; transl(T)'];

                   % animate the wrist frame
                   if ~isempty(h.wrist)
                       trplot(h.wrist, T);
                   end

                   % show the trajectory 
                   if(controller.display_opt.trajtrack)
                      for ii = 1:controller.references.GetNumTasks()
                         pos(:,ii) = controller.references.GetTraj(ii,time(i));
                         refreshdata(des_traj_pos(ii),'caller')
                         drawnow
                      end
                   end


                   % add a frame to the movie
                   if ~isempty(h.controller.subchains.framenum)
                       % write the frame to the movie folder
                       print( '-dpng', fullfile(h.controller.subchains.moviepath, sprintf('%04d.png', h.controller.subchains.framenum)) );
                       h.controller.subchains.framenum = h.controller.subchains.framenum+1;
                   end

                   if h.controller.subchains.delay > 0
                       pause(h.controller.subchains.delay);
                       drawnow
                   end
               end
               k = i;
            end
        end
        
        if ~h.controller.subchains.loop
            break;
        end        
    end
    
    h.q = q;
    set(group, 'UserData', h);