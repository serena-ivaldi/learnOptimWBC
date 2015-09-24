function [Aeq,beq] = EqualityConstraints(obj,M,F,DOF,projector_list)

    % i add 1 becaquse of the torque
    Aeq=zeros(DOF,DOF*(size(projector_list,1) + 1));
    Aeq(:,1:DOF)=eye(DOF);
    index = DOF;
    for i=1:size(projector_list,1)
        Aeq(:,index + 1:index + DOF) = -M*projector_list{i};
        index = index + DOF;    
    end
    beq = F;

end