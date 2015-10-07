function [meanerr,varerr] = ComputeErrorFromFig()


prompt1 = 'open the first image to acquire data and and click on it ';
disp(prompt1);
k = waitforbuttonpress 

% extract data line from fig
line=findobj(gca,'Type','line');

for i=1:length(line)
    XData=get(line(i),'XData'); %get the x data
    YData=get(line(i),'YData'); %get the y data
    data.X=XData; % row vector  
    data.Y=YData; % row vetcor

    Data1{i} = data;          
end


prompt2 = 'open the second image to acquire data and click on it ';
disp(prompt2);
k = waitforbuttonpress 

 % extract data line from fig
line=findobj(gca,'Type','line');

for i=1:length(line)
    XData=get(line(i),'XData'); %get the x data
    YData=get(line(i),'YData'); %get the y data
    data.X=XData; % row vector  
    data.Y=YData; % row vetcor

    Data2{i} = data;
end

set1=[];
set2=[];
% build the dataset to compare 
for i=1:length(line)
    set1= [set1;Data1{1,i}.Y];  %row matrix
    set2= [set2;Data2{1,i}.Y];  %row matrix
end



% compute the error vector
diff_mat =set1 - set2; 
diff_mat = diff_mat.*diff_mat;
result = sum(diff_mat,1);    % sum along rows
error = sqrt(result);


% compute mean and variance of error;
meanerr = mean(error);
varerr = var(error);








end