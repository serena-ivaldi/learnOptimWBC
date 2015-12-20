function violation = DistanceObs(input,constraints_values)
    global G_OB;
    violation = 0;
    for jj=1:size(G_OB,2)        
        dist = G_OB(jj).Dist(input,2);
        if(dist < constraints_values)
           violation = violation + (constraints_values - dist); 
        end  
    end

end