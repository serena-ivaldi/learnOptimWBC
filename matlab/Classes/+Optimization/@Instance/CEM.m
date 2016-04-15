function [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = CEM(instance,settings)
   plot_state = settings.plotState;
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
   optimizer = Optimization.CrossEntropy(instance,plot_state,n,maxAction,minAction,nIterations,explorationRate,settings.fnForwardModel);
   [performances,bestAction,BestActionPerEachGen,policies,costs,succeeded,G_data2save] = optimizer.CrossEntropyOptimzation();
end



