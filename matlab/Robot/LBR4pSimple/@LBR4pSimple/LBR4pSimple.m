classdef LBR4pSimple < SerialLink
 
	properties
	end
 
	methods
		function ro = LBR4pSimple()
			objdir = which('LBR4pSimple');
			idx = find(objdir == filesep,2,'last');
			objdir = objdir(1:idx(1));
			 
			tmp = load(fullfile(objdir,'@LBR4pSimple','matLBR4pSimple.mat'));
			 
			ro = ro@SerialLink(tmp.sr);
			 
			 
		end
	end
	 
end
