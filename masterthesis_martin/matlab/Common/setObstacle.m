function ob = setObstacle(vb_obj, tol)
    mgrid = vb_obj.mgrid;

    % change the representation of the meshgrid:
    mgrid_rep.X = mgrid.X(1,:,1);
    mgrid_rep.Y = mgrid.Y(:,1,1).';
    mgrid_rep.Z = permute(mgrid.Z(1,1,:), [2 3 1]);

    ob = Obstacle(mgrid_rep, 'wall', tol);
end
