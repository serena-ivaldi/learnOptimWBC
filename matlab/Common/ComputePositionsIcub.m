function [ee,elbow]=ComputePositionsIcub(q,t,controller)
ee = [];
elbow = [];
iCub = controller.GetWholeSystem();

for i=1:size(t,1)
    q_cur = q(i,:);    
    p1 = iCub.offlineFkine(q_cur','r_gripper');
    p2 = iCub.offlineFkine(q_cur','r_elbow_1');    
    ee = [ee , p1];
    elbow = [elbow, p2];
    
end
end
