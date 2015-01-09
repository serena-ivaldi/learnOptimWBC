function complete_path=PlotCmaesResult(time_struct,controller,bestAction,scriptname,name_folder)

% create folder 
allpath=which('FindData.m');
path=fileparts(allpath);
complete_path = strcat(path,'/results/',name_folder);
mkdir(complete_path)

% copy runtime parameter inside the new folder
rawTextFromStorage = fileread(which(scriptname));
rawTextFromStorage = regexp(rawTextFromStorage,['%%%;;' '(.*?)%%%EOF'],'match');
fileID = fopen(strcat(complete_path,'/','optimization_parameters.txt'),'w');

% write parameters
fprintf(fileID,'%s',rawTextFromStorage{1});
for z = 2:size(rawTextFromStorage,2)
    fseek(fileID, 0, 'eof');
    fwrite(fileID, rawTextFromStorage{z});
end
fclose(fileID);

% copy best action inside the new folder
fileID = fopen(strcat(complete_path,'/','best_action.txt'),'wt');
fprintf(fileID,'%f ',bestAction.parameters);

time = time_struct.ti:time_struct.step:time_struct.tf;
vec_values = zeros(size(time));
handle_vec = [];

% build numeric theta 
controller.UpdateParameters(bestAction.parameters)

% plot alpha 
for ii = 1:size(controller.alpha,1)
    for jj = 1:size(controller.alpha,2)
                    
        i=1;
        for t = time
            vec_values(i) = controller.alpha{ii,jj}.GetValue(t); 
            i=i+1;
        end

        handle_vec(ii,jj) = figure;
        plot(time,vec_values);
        xlabel('t','FontSize',16);
        ylabel(strcat('\alpha_{',num2str(ii),num2str(jj),'}'),'FontSize',16);
        
    end
end

% save alpha plot 
for ii = 1:size(controller.alpha,1)
    for jj = 1:size(controller.alpha,2)
    saveas(handle_vec(ii,jj),strcat(complete_path,'/','\alpha',num2str(ii),num2str(jj)),'jpg');
   end
end


% plot of fitnes function
evolutions = size(bestAction.hist,2);


% remove failed mean and failed variance
% -10000000 is the penalty that is applied when i have a failure 
index = 1;
for i =1:evolutions
    % i discard the evolutions with failure final mean
    if(bestAction.hist(1,i).performance>-10000000)
        listperformance = bestAction.hist(1,i).listperformance;
        %remove all the failure from the computation of the variance
        listperformance = listperformance(listperformance~=-10000000);
        variance(index) = var(listperformance);
        mean(index) = bestAction.hist(1,i).performance;
        evo(index) = index;
        index = index + 1; 
    end
    
end

handle = figure;
hold on;
plot(evo,mean);

%   fill([evo;flipud(evo)],[mean+2 * variance;flipud(mean+2 * variance)],...
%        'r','EdgeColor','r','FaceAlpha',0.1,'EdgeAlpha',0.3);
plot(evo, mean + 2 * variance, ':');
plot(evo, mean - 2 * variance, ':');
xlabel('evolutions','FontSize',16);
ylabel('fitness','FontSize',16);

saveas(handle,strcat(complete_path,'/','fit'),'jpg');

% figure
% alpha.PlotBasisFunction();
% 
% spani = 0:0.001:range(1,2);
% i = 1;
% sigvalue = zeros(1,size(spani,2));
% for x=spani
%     t = 0;
%     numeric_theta = x*ones(number_of_basis,1);
%     sigvalue(i) = feval(alpha.func,t,numeric_theta);
%     i= i + 1;
% end
% figure
% plot(spani,sigvalue)












end