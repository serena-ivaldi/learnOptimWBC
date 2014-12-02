function [dist]=MinDist(X,Y,Z,tp)


dist_matrix = (tp(1,1) - X).^2 + (tp(1,2) - Y).^2 + (tp(1,3) - Z).^2;
dist = min(dist_matrix(:));




end