clear all 
close all 
clc

%% global variable
variable_to_plot = 'G_distance_from_best_action';
box_to_plot_selector =[1 3]; % this vector specify wich element i want to plot from variable_to_plot
labels = {'a','c'};
colors = ['c','y']; % define the color of each box (inverse order)


%% path to dat file to open
%%  SAVE PATH
 % parameter
 folder = 'benckmark';
 subfolder = {'(1+1)CMAES-vanilla','CMAES-adaptive'};
 allpath=which('FindData.m');
 local_path=fileparts(allpath);
 
 %% LOAD DATA
 for i=1:length(subfolder)
    cur_mat = strcat(local_path,'/',folder,'/',subfolder{i},'/dat.mat');
    load(cur_mat,variable_to_plot);
    store_data{i} = eval(variable_to_plot);
 end
 
 
 %% create plot  
 % the hyphothesis is that each variable_to_plot come as matrix where the
 % lines are the box to plot while the columns are the elemnts of each box
 % plot
 % i make the hypothesis that the different matrix came with the same
 % amount of line but different column.
 
 % given the lenght of store_data and the number of element in box_to_plot_selector
 % we can compute the position for grouping the boxplot 
 len_store_data = length(store_data); % number of element per group
 len_box_to_plot_selector = length(box_to_plot_selector); % number of group
 
 step = 0.20;
 index = 1;
 for i=1:len_box_to_plot_selector
   for j = 0 : len_store_data-1
      positions(index) = i + j*step;
      index = index + 1;
   end
 end
 
%% create vector of data to plot and a the vector that specify to which box each value belong to
index = 1;
x = [];
group = [];
for i=1:len_box_to_plot_selector
   for j = 1:len_store_data
      x = [x store_data{j}(box_to_plot_selector(i),:)];
      group = [group , index * ones(1,size(store_data{j},2))];
      index = index + 1;
   end
end

%% plot boxplot
boxplot(x,group, 'positions', positions);

%% set label
% build the vector of mean position from positions 
index = 1;
for i = 1:len_box_to_plot_selector
   mean_pos(i) = mean(positions(index:(index + len_store_data - 1)));
   index = index + len_store_data;
end

set(gca,'xtick',mean_pos)
set(gca,'xticklabel',labels)


%% color box plot
% build the complete color vector
color_vec = [];
for i = 1:len_box_to_plot_selector
   color_vec = [color_vec colors];
end

h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color_vec(j),'FaceAlpha',.2);
end

%% put the legend
hleg1 = legend('Feature2', 'Feature1' );

