classdef Jaco < SerialLink
 
	properties
        robot2 % robot without friction
	end
 
	methods
		function ro = Jaco()
			objdir = which('Jaco');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@Jaco','matJaco.mat'));
			 
			ro = ro@SerialLink(tmp.sr);	 
        end
        
	end
	 
end
