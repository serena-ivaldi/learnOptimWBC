function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = CEO(instance,settings)
   nIterations = settings.nIterations;
   explorationRate = settings.explorationRate;
   if isfield(settings, 'activeIndices')
       action = settings.action(settings.activeIndices);
       minAction = settings.minAction(settings.activeIndices);
       maxAction = settings.maxAction(settings.activeIndices);
   else
       action = settings.action;
       minAction = settings.minAction;
       maxAction = settings.maxAction;
   end  
   n = size(action,2);
   optimizer = CrossEntropy(instance,n,maxAction,minAction,nIterations,explorationRate,);
   [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = optimizer.CrossEntropyOptimzation();
end



