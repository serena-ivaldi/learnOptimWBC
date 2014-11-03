% SubChains contiene naturalmente tutto il robot (non perturbato) e contenuto negli altri
% campi per cui subchain se non chiamo sub_chains si comporta come tutto il
% robot
classdef  SubChains < SerialLink


    properties
       % dim taget_links = ntask
       target_link; % vector that define wich kind of link i want to control with the e-e effector too
       sub_chains; % vector of sub kinematic chain of the robot respect of target link
    end
    
    
    methods
 
      % to copy the whole robot we can use the copy constructor of SerialLink 
      % r1 = SerialLink(r0)   
      function obj = SubChains(target_link,L,varargin)        
         
         %DEBUG
         %celldisp(varargin);
         %--
         obj = obj@SerialLink(L,varargin{:});
         obj.target_link = target_link;
      end
      
      
      
       function newobj = Perturb(obj,P)
          
          newobj = obj.perturb(P);
          %if target link is empty i define only the 
          n_task = obj.sub_chains.lenght();
          for i=1:n_task
             newobj.sub_chains(i) = obj.sub_chains(i).perturb(P);
          end
           
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
          end
          
          
          obj.sub_chains = vec_sub_chain;
          
       end
             
    end
    
    
    
end