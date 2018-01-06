function [lnk_traj, trg_pts] = setupTrajectories_liftObj(traj_conf, trg_conf)
    [nTraj, sz] = size(traj_conf);
    if (sz ~= 4)
        error('setupTrajectories_liftObj: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
    end
    [nTrg, sz] = size(trg_conf);
    if (sz ~= 3)
        error('setupTrajectories_liftObj: %s', WBM.wbmErrorMsg.WRONG_ARR_DIM);
    end

    % define and setup the trajectory curves:
    lnk_traj = repmat(WBM.wbmLinkTrajectory, nTraj, 1);
    for i = 1:nTraj
        lnk_traj(i,1).urdf_link_name = traj_conf{i,1};
        lnk_traj(i,1).jnt_annot_pos  = traj_conf{i,2};
        lnk_traj(i,1).ept_marker     = traj_conf{i,3};

        traj_color = traj_conf{i,4};
        lnk_traj(i,1).line_color = traj_color;
        lnk_traj(i,1).ept_color  = traj_color;
    end

    % define and setup the target points which
    % the controlled links have to reach:
    mkr_size1   = 10;
    mkr_size2   = 8;
    line_width1 = 1.2;
    line_width2 = 1.6;
    trg_pts     = repmat(WBM.wbmTargetPoint, nTrg, 1);
    for i = 1:nTrg
        trg_pts(i,1).pos       = (trg_conf{i,1}).';
        trg_pts(i,1).marker    = trg_conf{i,2};
        trg_pts(i,1).mkr_color = trg_conf{i,3};

        if strcmp(trg_pts(i,1).marker, 'o')
            trg_pts(i,1).mkr_size   = mkr_size1;
            trg_pts(i,1).line_width = line_width1;
        else
            trg_pts(i,1).mkr_size   = mkr_size2;
            trg_pts(i,1).line_width = line_width2;
        end
    end
end
