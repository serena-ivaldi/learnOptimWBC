function display(obj,q,time,stl)

% the number of time steps have to be same for each chain
% and has to be the same time sample how far as possible

% index k for downsampling visualization  
k = 1;
for j=1:size(time,2)
    %check if time fixed_step is active and the time step is sufficiently large 
    if(time(j)-time(k)>obj.display_opt.step)
        for i = 1:obj.subchains.GetNumChains();
            obj.SetCurRobotIndex(i);
            if(stl)
                obj.plot3d(q{i}(j,:),time(j))
            else
                obj.plot(q{i}(j,:),time(j))    
            end
                
        end
    k = i;
    end
end