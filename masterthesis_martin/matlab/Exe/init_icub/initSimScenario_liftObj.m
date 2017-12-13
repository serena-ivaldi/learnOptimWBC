function sim_config = initSimScenario_liftObj(wbm_icub, varargin)
    switch nargin
        case 3
            scn_mode   = varargin{1,1};
            show_light = varargin{1,2};
        case 2
            if ischar(varargin{1,1})
                scn_mode   = varargin{1,1};
                show_light = false;
            else
                show_light = varargin{1,1};
                scn_mode   = 'LightScn';
            end
        case 1
            % default values for the simulation ...
            scn_mode   = 'LightScn';
            show_light = false;
        otherwise
            error('initSimScenario_liftObj: %s', WBM.wbmErrorMsg.WRONG_NARGIN);
    end

    % Define the environmental scenario for the object lifting simulation:
    rotm_r     = eye(3,3); % rect. orientation
    vb_objects = repmat(WBM.vbCuboid, 3, 1);

    % general object properties ...
    obj_prop = struct('line_width', 0.4, 'edge_color', [], 'face_color', [], 'face_alpha', 0.2, ...
                      'description', '', 'ismovable', false, 'obj_type', 'obs', 'mesh_size', 0.01);

    % Geometric volume bodies:
    vb_alpha   = 0.5;                % transparency of the volume bodies
    furn_color = WBM.wbmColor.wheat; % furniture color
    cub_color  = WBM.wbmColor.khaki; % cube color

    % deposit table:
    tbl_prop  = obj_prop;
    tbl_prop.edge_color  = 'none';
    tbl_prop.face_color  = furn_color;
    tbl_prop.face_alpha  = vb_alpha;
    tbl_prop.description = 'deposit table';
    tbl_pos = [0.28; 0; 0.49]; % position of the tabletop (at CoM)
    %          table top size [m]: l_x, l_y, l_z
    vb_objects(1,1) = WBM.vbCuboid(0.40, 0.50, 0.02, tbl_pos, rotm_r, tbl_prop);

    % shelf:
    sh_prop  = obj_prop;
    sh_prop.edge_color  = 'none';
    sh_prop.face_color  = furn_color;
    sh_prop.face_alpha  = vb_alpha;
    sh_prop.description = 'shelf';
    sh_pos = [0.38; 0; 0.69]; % position of the shelf (at CoM)
    %              shelf size [m]: l_x, l_y, l_z
    vb_objects(2,1) = WBM.vbCuboid(0.20, 0.70, 0.02, sh_pos, rotm_r, sh_prop);

    % wooden cube (object to be grabbed and moved):
    cub_prop  = obj_prop;
    cub_prop.edge_color  = 'none';
    cub_prop.face_color  = cub_color;
    cub_prop.face_alpha  = vb_alpha;
    cub_prop.description = 'wooden cube';
    cub_prop.rho = 430; % density of a Norway spruce [kg/m^3], <https://www.engineeringtoolbox.com/wood-density-d_40.html>
    %cub_prop.rho = 740; % density of an European/English oak [kg/m^3], <https://www.engineeringtoolbox.com/wood-density-d_40.html>
    %cub_prop.rho = 720; % density of an European beech [kg/m^3], <https://en.wikipedia.org/wiki/Fagus_sylvatica>
    cub_pos = [0.20; 0; 0.56]; % position of the cube at its CoM
    l_s = 0.12; % side length [m]
    vb_objects(3,1) = WBM.vbCuboid(l_s, cub_pos, rotm_r, cub_prop);

    % Define the left and the right hand as payload links and
    % link both links to the wooden cube which will be grabbed:
    vbi_c = 3; % volume body index of the cube
    m_rb  = vb_objects(vbi_c,1).m_rb;
    I_cm  = vb_objects(vbi_c,1).I_cm;

    pl_lnk_l.name     = 'l_hand';
    pl_lnk_l.lnk_p_cm = cub_pos + [0; -l_s*0.5; 0];
    pl_lnk_l.m_rb     = m_rb;
    pl_lnk_l.I_cm     = I_cm;
    pl_lnk_l.vb_idx   = vbi_c;

    pl_lnk_r.name     = 'r_hand';
    pl_lnk_r.lnk_p_cm = cub_pos + [0; l_s*0.5; 0];
    pl_lnk_r.m_rb     = m_rb;
    pl_lnk_r.I_cm     = I_cm;
    pl_lnk_r.vb_idx   = vbi_c;

    pl_lnk_data = {pl_lnk_l, pl_lnk_r};
    wbm_icub.setPayloadLinks(pl_lnk_data);

    % Create and setup the simulation configuration:
    urdf_file_name = wbm_icub.robot_model.urdf_robot_name;
    sim_config     = initSimConfigICub_liftObj(urdf_file_name, vb_objects, scn_mode, show_light);
    % setup the payload stack to be processed ...
    % (the vb-index of the cube will be linked to both hands)
    sim_config.setPayloadStack(vbi_c, 'bh');
end
