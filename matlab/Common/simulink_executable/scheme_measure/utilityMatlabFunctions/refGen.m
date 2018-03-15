function [qDes,desired_x_dx_ddx_CoM] = refGen(xCom0,q0,t,references)
    qDes      = q0;
    xcomDes   = xCom0;
    xDcomDes  = zeros(3,1);
    xDDcomDes = zeros(3,1);

    for i = 1: size(references.joints.points,1)-1
        if t > references.joints.points(i,1) && t <= references.joints.points(i+1,1)
            qDes = references.joints.points(i,2:end)';
        end
    end


    if references.com.amplitudeOfOscillation ~= 0
        if t > references.com.noOscillationTime
            A = references.com.amplitudeOfOscillation;
        else
            A = 0;
        end
        f = references.com.frequencyOfOscillation;

        xcomDes    = xCom0 + A*sin(2*pi*f*t)*references.com.directionOfOscillation;
        xDcomDes   =         A*2*pi*f*cos(2*pi*f*t)*references.com.directionOfOscillation;
        xDDcomDes  =        -A*(2*pi*f)^2*sin(2*pi*f*t)*references.com.directionOfOscillation;

    else
        for i = 1: size(references.com.points,1)-1
            if t > references.com.points(i,1) && t <= references.com.points(i+1,1)
                xcomDes   = references.com.points(i,2:end)';
            end
        end
    end
    


    desired_x_dx_ddx_CoM = [xcomDes;xDcomDes;xDDcomDes];


end

