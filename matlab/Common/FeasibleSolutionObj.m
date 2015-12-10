function y = FeasibleSolutionObj(x,obj)
  [performances(1), succeeded(1), data2save] = obj.EvaluateCMAES(x,-1,1);
   obj.penalty_handling.penalties(1,:)>0; % vector of index of the violated constrained 
   y = sum(obj.penalty_handling.penalties(1,:).^2,2);
end
