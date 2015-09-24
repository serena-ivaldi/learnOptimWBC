function WriteFF(p_tot,TaskSpaceDim,namefile)

   fileID = fopen(namefile,'w');
   % here i write the kind of control that i want to do on the kinova
   fprintf(fileID,'%1.0f \n',TaskSpaceDim);

   w_string =''; 
   particle = '%3.5f ';
   for ii = 1 : TaskSpaceDim
       w_string = [w_string , particle];
   end
   w_string = strcat(w_string,'\n');
   
   for i = 1:size(p_tot,1)
      fprintf(fileID,w_string,p_tot(i,:));
   end
   fclose(fileID);
end