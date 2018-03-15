function H = homTranformFromAxisAngle(u)
%#codegen
%%Must check that euler angles are x-y-z
H =  [ rotationFromAxisAngle(u(4:7)),u(1:3)
          zeros(1,3),1]  ;
end
