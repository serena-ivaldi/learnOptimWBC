% SubChains naturally contains the robot without perturbation
% SubChains became really usefull when i have DIFERRENT kinematic chains
classdef  SubChains < SerialLink


    properties
       dyn_model;    % full dynamical model of the robot
       target_link;  % vector that define wich kind of link i want to control with the e-e effector too (row vector)
       sub_chains;   % vector of sub kinematic chain of the robot respect of target link (in general pertubed)
       sub_chainsGT; % vector of subchain non perturbed
    end
    
    
    methods
 
      % to copy the whole robot we can use the copy constructor of SerialLink 
      % r1 = SerialLink(r0)   
      function obj = SubChains(target_link,L,model,varargin)        
         
         %DEBUG
         %celldisp(varargin);
         %--
         obj = obj@SerialLink(L,varargin{:});
         obj.dyn_model = model;
         obj.target_link = target_link;
      end
      
      
       % with this function i obtain a pertuberb copy of my SubChain
       function newobj = Perturb(obj,P)
          
          %the object contained in  SubChains is the model so i don't have
          %to perturb it
          %newobj = obj.perturb(P);
          %if target link is empty i define only the 
          n_task = obj.sub_chains.lenght();
          for i=1:n_task
             newobj.sub_chains(i) = obj.sub_chains(i).perturb(P);
          end
           
       end
       
       function n=GetNumTasks(obj)
          n=size(obj.target_link,2);
       end 
       
       function n=GetNumLinks(obj)
          n=size(obj.links,2);
       end 
       
       function n=GetNumSubLinks(obj,index)
          n=size(obj.sub_chains(index).links,2);
       end 
       
      
    end
    
     
    methods (Static)
       function obj = BuildSC(target_link,L,P,varargin)
          
          obj = SubChains(target_link,L,varargin{:});
          %if target link is empty i define only the 
          n_task = size(target_link,2);
          for i=1:n_task
             sub_chain = L(1:target_link(i));
             app_subchains = SubChains(target_link,sub_chain,varargin{:});
              vec_sub_chain(i)= app_subchains.perturb(P);
              % actually i copy the subchain through the perturb function
              % but with P = 0 and i will use it as a ground truth
              vec_sub_chainGT(i) = app_subchains.perturb(0);
          end
          
          
          obj.sub_chains   = vec_sub_chain;
          obj.sub_chainsGT = vec_sub_chainGT;
          
       end
             
    end
    
    
    
end