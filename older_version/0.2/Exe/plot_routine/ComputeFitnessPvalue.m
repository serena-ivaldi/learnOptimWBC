
function  [h p]=ComputeFitnessPvalue

list_of_folder = {'23_sere','22_sere'};
list_of_subfolder1 = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20];
list_of_subfolder2 = [1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20];
list_of_subfolder = {list_of_subfolder1,list_of_subfolder2};

% collect best fit;
for i=1:1:size(list_of_folder,2)
    bestfit=[];
    for j=1:size(list_of_subfolder{1,i},2)
        cur_folder_path = BuildMatFilePath(list_of_folder{i},list_of_subfolder{1,i}(1,j));
        bestfit = [bestfit GetBestFit(cur_folder_path)];
    end
    allfit{i}= bestfit;
end

%  T student test

[h p] = kstest2(allfit{1},allfit{2});

mean1 = mean(allfit{1})
std1  = std(allfit{1})

mean2 = mean(allfit{2})
std2  = std(allfit{2})

 
end


% this function give back the path to .mat file of the current experiment 
function result=BuildMatFilePath(folder_name,experiment_index)

   allpath=which('FindData.m');
   path=fileparts(allpath);
   result = strcat(path,'/results/',folder_name,'/',num2str(experiment_index),'_of_',folder_name,'/data.mat');

end



function [best_fit] = GetBestFit(cur_mat_path)

   load(cur_mat_path);
   % i need to multiply per 0.001 because the fitness has a different scale 
   %best_fit = bestAction.hist(1, 79).performance;
   best_fit = bestAction.performance*0.001;
    
end