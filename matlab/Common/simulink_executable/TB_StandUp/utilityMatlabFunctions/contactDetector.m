function legsInContact = contactDetector(icubStandup,state)
%#codegen
legsInContact = 0;

if (state < 3) && (icubStandup == 1)
    
    legsInContact = 1;
    
end

end
