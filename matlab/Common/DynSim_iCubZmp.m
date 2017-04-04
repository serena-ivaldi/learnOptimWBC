function [zmp,CoP]=DynSim_iCubZmp(fc,param,x_b,w_R_b,q)
    import WBM.utilities.frame2posRotm; 
    contact_sym = param.contact_sym;
    
    % centers of pressure at feet
    if  contact_sym.num_of_active_contacts == 1
    CoP(1)         = -fc(5)/fc(3);
    CoP(2)         =  fc(4)/fc(3);
    elseif  contact_sym.num_of_active_contacts >= 2
        CoP(1)     = -fc(5)/fc(3);
        CoP(2)     =  fc(4)/fc(3);
        CoP(3)     = -fc(11)/fc(9);
        CoP(4)     =  fc(10)/fc(9);
    end
    
    if contact_sym.state(1) == 1 && contact_sym.state(2) == 0 %&& contact_sym.state(3) == 0 && contact_sym.state(4) == 0

        zmp = CoP;

    elseif contact_sym.state(1) == 0 && contact_sym.state(2) == 1 %&& contact_sym.state(3) == 0  && contact_sym.state(4) == 0

        zmp = CoP;

    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 %&& contact_sym.state(3) == 0  && contact_sym.state(4) == 0
        
        % the cop in the right foot is expressed in the local frame of the
        % foot so we need to express it in the world referecence frame
        r_sole_pos = wbm_forwardKinematics(w_R_b,x_b,q,'r_sole');
        r_sole_position = r_sole_pos(1:2)';
        [~,w_R_rsole]    = frame2posRotm(r_sole_pos);
        
        w_rsole_cop = w_R_rsole(1:2,1:2)*CoP(3:4)' + r_sole_position';
        
        
        zmp = (fc(3)*CoP(1:2) + fc(9)*w_rsole_cop')/(fc(3)+fc(9));
        
    else
        % here there is something wrong if it happens
        zmp = [-1000,-1000];
        CoP = [nan , nan  ];
        
    end




end