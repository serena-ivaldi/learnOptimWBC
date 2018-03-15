function dotNu_ikin   = Ikin_AccelerationRef_generator(Jc, Jcom, Jp, dJcNu, dJcomNu,iKinFeetCorr, iKinComCorr, iKinPostCorr)

%setup parameters
n_joints  = length(Jcom(1,7:end));
PINV_TOL  = 5e-7;  

% null space projectors for primary and secondary task
Nc   = eye(6+n_joints) - pinv(Jc,PINV_TOL)*Jc;
Ncom = eye(6+n_joints) - pinv(Jcom*Nc,PINV_TOL)*Jcom*Nc;

%reference acceleration calculation
dotNu_feet   = pinv(Jc,PINV_TOL)*(iKinFeetCorr - dJcNu);

dotNu_com    =  pinv(Jcom*Nc, PINV_TOL)*(iKinComCorr - dJcomNu - Jcom*dotNu_feet);

dotNu_post   = pinv(Jp*Nc*Ncom, PINV_TOL)*(iKinPostCorr - Jp*dotNu_feet -Jp*Nc*dotNu_com);

dotNu_ikin   = dotNu_feet + Nc*(dotNu_com + Ncom*dotNu_post);

end