classdef Contacts < handle
    properties
        state                  % here i specify if the contact is active or not (1 or 0)
        names                  % here i specify wich is the point of cotact between the robot and the enviroment
        num_of_active_contacts 
    end
    
    methods
        function obj=Contacts(active,names)
            obj.state = active;
            obj.names = names;
            obj.num_of_active_contacts = obj.GetActiveContact();
        end
        
        function res=GetActiveContact(obj)
            res = sum(obj.state,2);
        end
        
        function res = UpdateContact(obj)
            res = obj.GetActiveContact();
           obj.num_of_active_contacts = res;
        end
    end
    
    
    
end