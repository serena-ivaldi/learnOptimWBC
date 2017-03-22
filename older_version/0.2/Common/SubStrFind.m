function C = SubStrFind(x,y) 
     C = ~cellfun(@isempty,strfind(y,x));
end