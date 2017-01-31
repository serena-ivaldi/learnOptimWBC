%% in this case output is the candidate because Im dealing with simple optimization problem
%% for benchmarking purpose

%% i have to remove the random part from this function :)
function fit = to_test_withBOGP_stuff(obj,output)
   % here i fix the random generator to assure reproducibility of the results
   %rng(5.0);
   %% TODO extend this input check to the other fitness function if necessary
   if(~isempty(obj))
        input_vector = obj.CreateInputFromParameters(output);
        obj.penalty_handling.EvaluateConstraints(input_vector,1);
   end
   % i want to minimize so i invert the sign of the function used in the
   % GP_stuff example
   %fit =  log( (mvnpdf([output(:,1) output(:,2)],[-1.5 -2.5], [1 0.3; 0.3 1]) + 0.3*mvnpdf([output(:,1) output(:,2)],[2 3], [3 0.5; 0.5 4])).*...
   %mvnpdf([output(:,1) output(:,2)],[0 0], [100 0; 0 100])) ./15 - 1;

   fit = +log( (mvnpdf([output(:,1) output(:,2)],[3.5 2.5], [1 0.3; 0.3 1]) + 0.3*mvnpdf([output(:,1) output(:,2)],[7 8], [3 0.5; 0.5 4])).*...
        mvnpdf([output(:,1) output(:,2)],[5 5], [100 0; 0 100])) ./15 +1;
end