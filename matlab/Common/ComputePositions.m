function [ee,elbow]=ComputePositions(q,t,controller)
   ee = [];
   elbow = [];
   
   for i=1:size(t,2)
      q_cur = q(i,:);
      % compute the trajectory error (absolute error)
      kinematic=CStrCatStr({'controller.subchains.sub_chains{1}.T0_'},num2str(controller.subchains.GetNumSubLinks(1,1)),{'(q_cur)'});
      T = eval(kinematic{1});
      ee = [ee , T(1:3,4)];
      kinematic=CStrCatStr({'controller.subchains.sub_chains{1}.T0_'},'3',{'(q_cur)'});
      T = eval(kinematic{1});
      elbow = [elbow, T(1:3,4)];
   end
end