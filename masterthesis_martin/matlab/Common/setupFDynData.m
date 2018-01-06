function fdyn_data = setupFDynData(bot, cstate_foot, cstate_hand, ac_f, mu_s)
    rtype = 'eul';
    [~,q_j,~,~] = getState(bot);

    foot_conf = footConfig(bot, cstate_foot, q_j, rtype);
    hand_conf = handConfig(bot, cstate_hand, 'hand', q_j, rtype);

    % function handle to the contact wrench function for the hands ...
    fhTotCWrench = @(f_cp, lnk_R_cm, lnk_p_cm)cwrenchHand(f_cp, lnk_R_cm, lnk_p_cm, mu_s);

    % calculate the applied forces at the contact points (cp):
    g_wf  = bot.hwbm.g_wf;  % gravity vector in the world frame (in [N])
    m_cub = bot.sim_config.environment.vb_objects(3,1).m_rb; % mass of the wooden cube in [kg]

    f_y   = m_cub .* g_wf;  % total force (in [N]) against the payload object's gravity force
                            % f_pg = m_cub * g,  s.t.  f_y = -f_pg = ft_c1 + ft_c2
                            % (along the y-axis and at the CoM of the object)

    ft_c  = f_y(3,1) * 0.5; % tangential force (in [N]) at each contact frame {c_i} of the hands
                            % (along the y-axis and against the gravity force of the object)

    fn_c  = ft_c / mu_s;    % normal force (in [N]) at each contact frame {c_i} of the hands
                            % (along the z-axis and in direction to the CoM of the object)


    % occurring forces (in [N]) at each contact frame {c_i} of the hands:
    f_c = zeros(3,2);
    %      axis:  x     y      z
    f_c(1:3,1) = [0;  ft_c; -fn_c]; % left hand
    f_c(1:3,2) = [0;  ft_c;  fn_c]; % right hand

    % to calculate the friction cone FC, split up the tangential force
    % vector ft_c in its tangential components ...
    f_s = cos(mu_s) * ft_c;
    f_t = sin(mu_s) * ft_c;

    sf = 1.15; % safety factor for grasping

    % forces applied to the object at contact point pc_i in frame {c_i}
    % of each hand:
    %
    % Note: If both hands are used for grasping, then the force vector f_cp
    %       must be a vector of even length of concatenated forces and each
    %       force of the vector must have at least one tangential force as
    %       component. The z-axis of each contact frame {c_i} points in the
    %       direction of the inward surface normal at the point of contact.
    %
    %         left hand          |   right hand
    % axis:   x,   y,   z        |   x,   y,   z
    f_cp = [f_s, f_t, f_c(3,1)*sf, f_s, f_t, f_c(3,2)*sf]; % in [N]

    fdyn_data = struct('fhTotCWrench', fhTotCWrench, 'foot_conf', foot_conf, ...
                       'hand_conf', hand_conf, 'f_cp', f_cp, 'ac_f', ac_f);
end
%% END of setupFDynData.


%% CONTACT WRENCH FUNCTION:

function wc_lnk = cwrenchHand(f_cp, lnk_R_cm, lnk_p_cm, mu_s)
    cm_R_lnk = lnk_R_cm; % = I_3

    % calculation of the transformation from the contact
    % frame {c} = {lnk} to the object frame {cm}:
    %
    %    c_R_lnk  = R_x(pi/2)
    %    lnk_R_c  = c_R_lnk^T = R_x(-pi/2)
    %    cm_R_lnk = lnk_R_cm = I_3
    %
    %    cm_R_lnk = cm_R_c * c_R_lnk
    %    cm_R_c   = cm_R_lnk * lnk_R_c
    %
    %    c_p_lnk = lnk_p_c = 0 --> lnk = c
    %    c_p_cm  = p_c = lnk_p_cm
    %    c_p_cm  = c_R_lnk * lnk_p_cm
    %    --> cm_p_c = cm_R_c * p_c = -c_p_cm
    cm_R_c =  cm_R_lnk * WBM.utilities.tfms.rotx(-pi*0.5);
    cm_p_c = -lnk_p_cm;

    % negated contact wrench (applied to the object at
    % contact point cm_p_c in contact frame {c} = {lnk}):
    wc_lnk = -WBM.utilities.mbd.cwrenchPalm(f_cp, cm_R_c, cm_p_c, mu_s);
end
