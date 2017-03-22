classdef LBR4p < SerialLink
 
	properties
	end
 
	methods
		function ro = LBR4p()
			objdir = which('LBR4p');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@LBR4p','matLBR4p.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
