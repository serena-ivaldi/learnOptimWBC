classdef LBR4pReal < SerialLink
 
	properties
	end
 
	methods
		function ro = LBR4pReal()
			objdir = which('LBR4pReal');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@LBR4pReal','matLBR4pReal.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
