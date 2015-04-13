function [Jdot] = genJdot(CGen)
   %% Derivation of symbolic expressions
   CGen.logmsg([datestr(now),'\tDeriving Jdot']);

  [q,qd] = CGen.rob.gencoords;
   Jdot = CGen.rob.jacob_dot(q,qd);

   CGen.logmsg('\t%s\n',' done!');

   %% Save symbolic expressions
   if CGen.saveresult
       CGen.logmsg([datestr(now),'\tSaving symbolic Jdot']);

       CGen.savesym(Jdot,'jacob_dot','jacob_dot.mat')

       CGen.logmsg('\t%s\n',' done!');

   end

   %% M-Functions
   if CGen.genmfun
       CGen.genmfunJdot;
   end

%    %% Embedded Matlab Function Simulink blocks
%    if CGen.genslblock
%        genslblockfkine(CGen);
%    end

   %% C-Code
   if CGen.genccode
       CGen.genccodeJdot;
   end

   %% MEX
   if CGen.genmex
       CGen.genmexJdot;
   end



end