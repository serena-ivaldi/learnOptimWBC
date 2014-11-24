function display(obj,q,stl)

% the number of torque as to be same for each chain
for t=1:size(q{1},1)
    
    for i = 1:obj.subchains.GetNumChains();
        
        obj.SetCurRobotIndex(i);
        
        if(stl)
            obj.plot3d(q{i}(t,:))
        else
            obj.plot(q{i}(t,:))    
        end
    end
    
end