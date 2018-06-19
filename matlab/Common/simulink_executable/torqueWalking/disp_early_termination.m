% if exist('exitFlagQP_unsuccessful', 'var')
%     if exitFlagQP_unsuccessful.Data(end)
%         disp("Early termination due to unsuccessful QP");
%     elseif no_force_on_feet.Data(end)
%         disp("Early termination due to no forces on feet");
%     elseif exist('ZMP_out_of_bounds', 'var')
%         if(~isempty(ZMP_out_of_bounds.Data(end)))
%             if ZMP_out_of_bounds.Data(end)
%                 disp("Early termination due to ZMP out of support polygon");
%             else
%                 disp("Normal termination at end of simulation time");
%             end
%         end
%     else
%         disp("Normal termination at end of simulation time");
%     end
% end
% 
% if exist('results', 'var')
%     if results.exitFlagQP_unsuccessful.Data(end)
%         disp("Early termination due to unsuccessful QP");
%     elseif results.no_force_on_feet.Data(end)
%         disp("Early termination due to no forces on feet");
%     elseif exist('results.ZMP_out_of_bounds', 'var')
%         if results.ZMP_out_of_bounds.Data(end)
%             disp("Early termination due to ZMP out of support polygon");
%         end
%     else
%         disp("Normal termination at end of simulation time");
%     end
% end