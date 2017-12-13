function [lnk_traj, trg_pts] = setupTrajectories_liftObj(trg_pos)
    [nTrg, sz] = size(trg_pos);
    if (sz ~= 3)
        error('setupTrajectories_liftObj: %s', WBM.wbmErrorMsg.WRONG_MAT_DIM);
    end

    % define and setup the trajectory curves for the left and the right hand:
    lnk_traj = repmat(WBM.wbmLinkTrajectory, 2, 1);
    lnk_traj(1,1).urdf_link_name = 'l_hand';
    lnk_traj(2,1).urdf_link_name = 'r_hand';

    idx_j = 7; % joint index
    lnk_traj(1,1).jnt_annot_pos = {'left_arm', idx_j};
    lnk_traj(2,1).jnt_annot_pos = {'right_arm', idx_j};

    traj_color_l = WBM.wbmColor.forestgreen;
    traj_color_r = WBM.wbmColor.tomato;

    lnk_traj(1,1).line_color = traj_color_l;
    lnk_traj(1,1).ept_color  = traj_color_l;

    lnk_traj(2,1).line_color = traj_color_r;
    lnk_traj(2,1).ept_color  = traj_color_r;

    % define and setup the final target points which the hands have to reach:
    trg_pts    = repmat(WBM.wbmTargetPoint, nTrg, 1);
    line_width = 1.5;
    mkr_size   = 10;
    mkr_color1 = WBM.wbmColor.turquoise1;
    mkr_color2 = WBM.wbmColor.deeppink;

    for i = 1:nTrg
        trg_pts(i,1).pos        = trg_pos(i,1:3).';
        trg_pts(i,1).mkr_size   = mkr_size;
        trg_pts(i,1).line_width = line_width;
    end

    if (nTrg == 4)
        % set the colors of the first 2 target point pairs:
        % first (intermediate) targets:
        trg_pts(1,1).mkr_color = mkr_color1;
        trg_pts(2,1).mkr_color = mkr_color1;
        % final targets:
        trg_pts(3,1).mkr_color = mkr_color2;
        trg_pts(4,1).mkr_color = mkr_color2;
    end
end
