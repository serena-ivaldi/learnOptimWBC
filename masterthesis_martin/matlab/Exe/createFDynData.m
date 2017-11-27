function fdyn_data = createFDynData(bot, cstate_foot, cstate_hand, f_cp, lnk_R_cp, lnk_p_cp, mu_s)
    rtype = 'eul';
    [~,q_j,~,~] = bot.getState();

    foot_conf = bot.footConfig(cstate_foot, q_j, rtype);
    hand_conf = bot.handConfig(cstate_hand, q_j, rtype);
    fhTotCWrench = WBM.utilities.mbd.cwrenchPalm(f_cp, lnk_R_cp, lnk_p_cp, mu_s);

    fdyn_data = struct('fhTotCWrench', fhTotCWrench, 'foot_conf', foot_conf, ...
                       'hand_conf', hand_conf, 'f_cp', f_cp, 'ac_f', 0);
end
