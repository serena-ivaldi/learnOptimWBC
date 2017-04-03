function [zmp]=DynSim_iCubZmp(fc,param)

    contact_sym = param.contact_sym;
    
    % centers of pressure at feet
    if  contact_sym.num_of_active_contacts == 1
    CoP(1)         = -fc(5)/fc(3);
    CoP(2)         =  fc(4)/fc(3);
    elseif  contact_sym.num_of_active_contacts == 2
        CoP(1)     = -fc(5)/fc(3);
        CoP(2)     =  fc(4)/fc(3);
        CoP(3)     = -fc(11)/fc(9);
        CoP(4)     =  fc(10)/fc(9);
        
    end
    
    if contact_sym.state(1) == 1 && contact_sym.state(2) == 0 && contact_sym.state(3) == 0 && contact_sym.state(4) == 0

        zmp = CoP;

    elseif contact_sym.state(1) == 0 && contact_sym.state(2) == 1 && contact_sym.state(3) == 0  && contact_sym.state(4) == 0

        zmp = CoP;

    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 && contact_sym.state(3) == 0  && contact_sym.state(4) == 0
        
        zmp = (fc(3)*CoP(1:2) + fc(9)*CoP(3:4))/(fc(3)+fc(9));
        
    else
        % in the case when the robot is sitting i make the hyp that the
        % robot is in balance
        zmp = [+1000,1000];
        
    end




end