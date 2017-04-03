function [tauContact,fc]=DynSim_iCubContacts(ndof,state,dynamic,Jc,dJcNu,param,tau)
    import WBM.utilities.frame2posRotm;
    import WBM.utilities.rotm2eulAngVelTF
    Jc_t = Jc';
    M = dynamic.M;
    h = dynamic.h;
    q = state.q;
    w_R_b = state.w_R_b;                      
    x_b = state.x_b;
    Nu = state.Nu;
    contact_sym = param.contact_sym;
    
    %% moving floating base
    % feet correction gain
    K_corr_pos  = 100;
    K_corr_vel  = 2*sqrt(K_corr_pos);
    % feet current position and orientation
    l_sole       = wbm_forwardKinematics(w_R_b,x_b,q,'l_sole');
    r_sole       = wbm_forwardKinematics(w_R_b,x_b,q,'r_sole');
    l_upper_leg  = wbm_forwardKinematics(w_R_b,x_b,q,'l_upper_leg');
    r_upper_leg  = wbm_forwardKinematics(w_R_b,x_b,q,'r_upper_leg');
    [x_lfoot,R_b_lfoot]              = frame2posRotm(l_sole);
    [x_rfoot,R_b_rfoot]              = frame2posRotm(r_sole);
    [x_lupper_leg,R_b_lupper_leg]    = frame2posRotm(l_upper_leg);
    [x_rupper_leg,R_b_rupper_leg]    = frame2posRotm(r_upper_leg);
    % orientation is parametrized with euler angles
    [phi_lfoot,TLfoot]          = rotm2eulAngVelTF(R_b_lfoot);
    [phi_rfoot,TRfoot]          = rotm2eulAngVelTF(R_b_rfoot);
    [phi_luleg,TLUleg]          = rotm2eulAngVelTF(R_b_lupper_leg);
    [phi_ruleg,TRUleg]          = rotm2eulAngVelTF(R_b_rupper_leg);
    
    pos_leftFoot           = [x_lfoot; phi_lfoot];
    pos_rightFoot          = [x_rfoot; phi_rfoot];
    pos_LeftUpperLeg       = [x_lupper_leg;phi_luleg];
    pos_RightUpperLeg       = [x_rupper_leg;phi_ruleg];
 
    TL   = [eye(3) zeros(3) ; zeros(3) TLfoot];
    TR   = [eye(3) zeros(3) ; zeros(3) TRfoot];
    TLUL = [eye(3) zeros(3) ; zeros(3) TLUleg];
    TRUL = [eye(3) zeros(3) ; zeros(3) TRUleg];
    %% TODO we should move this part outside (this value are fixed)
    %    feet original position and orientation 
    lsole_ini              = param.lfoot_ini;
    rsole_ini              = param.rfoot_ini;
    lu_leg_ini             = param.lu_leg_ini;
    ru_leg_ini             = param.ru_leg_ini;

    [xi_lfoot,R_bi_lfoot]  = frame2posRotm(lsole_ini);
    [xi_rfoot,R_bi_rfoot]  = frame2posRotm(rsole_ini);
    [xi_luleg,R_bi_luleg]  = frame2posRotm(lu_leg_ini);
    [xi_ruleg,R_bi_ruleg]  = frame2posRotm(ru_leg_ini);

    [phi_rfoot_ini,~]      = rotm2eulAngVelTF(R_bi_rfoot);
    [phi_lfoot_ini,~]      = rotm2eulAngVelTF(R_bi_lfoot);
    [phi_luleg_ini,~]      = rotm2eulAngVelTF(R_bi_luleg);
    [phi_ruleg_ini,~]      = rotm2eulAngVelTF(R_bi_ruleg);

    lfoot_ini_tot          = [xi_lfoot; phi_lfoot_ini];
    rfoot_ini_tot          = [xi_rfoot; phi_rfoot_ini];
    luleg_ini_tot          = [xi_luleg; phi_luleg_ini];
    ruleg_ini_tot          = [xi_ruleg; phi_ruleg_ini];
    
    deltaPoseLFoot        = TL*(pos_leftFoot-lfoot_ini_tot);    
    deltaPoseRFoot        = TR*(pos_rightFoot-rfoot_ini_tot);
    deltaPoseLULeg        = TLUL*(pos_LeftUpperLeg-luleg_ini_tot);
    deltaPoseRULeg        = TRUL*(pos_RightUpperLeg-ruleg_ini_tot);


    % error between original and current feet position and orientation
    if contact_sym.state(1) == 1 && contact_sym.state(2) == 0 && contact_sym.state(3) == 0 && contact_sym.state(4) == 0

        pos_feet_delta =  deltaPoseLFoot;

    elseif contact_sym.state(1) == 0 && contact_sym.state(2) == 1 && contact_sym.state(3) == 0  && contact_sym.state(4) == 0

        pos_feet_delta =  deltaPoseRFoot;

    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 && contact_sym.state(3) == 0  && contact_sym.state(4) == 0
        
        pos_feet_delta = [deltaPoseLFoot;deltaPoseRFoot];
        
    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 && contact_sym.state(3) == 1  && contact_sym.state(4) == 1
        
        pos_feet_delta = [deltaPoseLFoot;deltaPoseRFoot;deltaPoseLULeg;deltaPoseRULeg];
        
    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 && contact_sym.state(3) == 1  && contact_sym.state(4) == 0
        
        pos_feet_delta = [deltaPoseLFoot;deltaPoseRFoot;deltaPoseLULeg];
        
    elseif contact_sym.state(1) == 1 && contact_sym.state(2) == 1 && contact_sym.state(3) == 0  && contact_sym.state(4) == 1
        
        pos_feet_delta = [deltaPoseLFoot;deltaPoseRFoot;deltaPoseRULeg];
    end
    %% Compute contact forces
    % % Real contact forces computation
    S               = [ zeros(6,ndof); eye(ndof,ndof)];
    JcMinv          = Jc/M;
    JcMinvS         = JcMinv*S;
    % the way which fc is calculated has to be checked
    if(sum(contact_sym.state)>2)
        delta = 0.001;
        JcMinvJc_t = JcMinv*Jc_t;
        One = eye(size(JcMinvJc_t,1));
        fc              = (JcMinv*Jc_t + One*delta)\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);
    else
        fc              = (JcMinv*Jc_t)\(JcMinv*h -JcMinvS*tau -dJcNu -K_corr_vel.*Jc*Nu -K_corr_pos.*pos_feet_delta);
    end
    
    tauContact = Jc_t*fc;
    
    
end