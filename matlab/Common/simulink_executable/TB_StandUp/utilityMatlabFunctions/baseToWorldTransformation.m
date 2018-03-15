function w_H_b_updated = baseToWorldTransformation(w_H_l,w_H_r,w_H_lr,w_H_b,linkToComputeBase)
%#codegen
    
    l_H_b         = w_H_l\w_H_b;

    r_H_b         = w_H_r\w_H_b;

    
    if linkToComputeBase(1) == 1
        w_H_b_updated = w_H_lr*l_H_b;
    elseif linkToComputeBase(2) == 1
        w_H_b_updated = w_H_lr*r_H_b;
    else
        w_H_b_updated = zeros(4);
    end
    
end