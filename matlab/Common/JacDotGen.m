function JacDotGen(obj,path)

   n=obj.n;
   path = strcat(path,'/','jacob_dot');
   ob = sym('ob');
   q = sym('q', [1,n]);
   qd = sym ('qd',[1,n]);

   J_dot=obj.jacob_dot(q,qd);

   matlabFunction(J_dot,'file',path,'vars', {ob,q,qd});
   


end