function output = ComputePositionsIcub(q,t,controller,tags)
output = [];
iCub = controller.GetWholeSystem();

for ii = 1:length(tags)
    for i=1:size(t,1)
        q_cur = q(i,:);
        p = iCub.offlineFkine(q_cur',tags{ii});
        output = [output , p];
    end
end
end

% function [ee,elbow]=ComputePositionsIcub(q,t,controller)
% ee = [];
% elbow = [];
% iCub = controller.GetWholeSystem();
% 
% for i=1:size(t,1)
%     q_cur = q(i,:);    
%     p1 = iCub.offlineFkine(q_cur','r_wrist_1');
%     p2 = iCub.offlineFkine(q_cur','r_elbow_1');    
%     ee = [ee , p1];
%     elbow = [elbow, p2];
%     
% end
% end

