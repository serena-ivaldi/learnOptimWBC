classdef ConvexHull
    %CONVEXHULL Summary of this class goes here
    %   use convhull and inpolygon functions from matlab
    
    properties
        K   % The 2-D convex hull of the points (X,Y), where X and Y are column vectors. 
            % The convex hull K is expressed in terms of a vector of point indices 
            % arranged in a counterclockwise cycle around the hull
        X   % X coordinates of the polygon points can be row or column
        Y   % Y coordinate of the polygon points can be row or column
        
    end
    
    methods
        function obj = ConvexHull(X,Y)
        %Constructor
        %X and Y are  list of coordinates (can be row or column)
        
            %Error checking
            assert(all(size(X(:))==size(Y(:))),'Input arguments: x & y are expected to be the same length');
            assert(~ all(size(X(:)) == [1 1]),'Input arguments: x should not be a singelton');
            assert(~ all(size(Y(:)) == [1 1]),'Input arguments: y should not be a singelton');

            obj.X = X;
            obj.Y = Y;
            obj.K = convhull(X,Y);
        end
        
        
        function [Xc, Yc] = getCenter(obj)
        % Return the coordinate of the center of the convex hull
            
            A = obj.X(obj.K(1:end-1)).*obj.Y(obj.K(2:end))- obj.X(obj.K(2:end)).*obj.Y(obj.K(1:end-1));
            As = sum(A)/2;
            Xc = (sum((obj.X(obj.K(2:end))+obj.X(obj.K(1:end-1))).*A)*1/6)/As;
            Yc = (sum((obj.Y(obj.K(2:end))+obj.Y(obj.K(1:end-1))).*A)*1/6)/As;
        end
        
        
        function dist = computeDistance(obj, Xq, Yq)
        % Return zero if the point is inside the hull or the distance
        % between the query point and the center of the hull if outside
            
            %Error checking
            assert(all(size(Xq(:)) == [1 1]),'Input arguments: Xq should be a singelton');
            assert(all(size(Yq(:)) == [1 1]),'Input arguments: Yq should be a singelton');
            
            in = inpolygon(Xq,Yq,obj.X(obj.K),obj.Y(obj.K));
            if (in) 
                dist = 0;
            else
                [Xc, Yc] = obj.getCenter;
                dist = sqrt((Xq - Xc)^2 + (Yq - Yc)^2);
            end
        end
        
        function  plotConvHull(obj,figHandle,Xq,Yq)
        % Plot the shape of the support polygon, the characteristic points
        % of the support(s) and the position of the query point (CoM, CoP,
        % ZMP, ...)
            %Error checking
            %assert(all(size(Xq(:)) == [1 1]),'Input arguments: Xq should be a singelton');
            %assert(all(size(Yq(:)) == [1 1]),'Input arguments: Yq should be a singelton');

            %figHandle = figure;
            figure(figHandle);
            hold on
            plot(obj.X,obj.Y,'b*');
            [Xc, Yc] = obj.getCenter;
            plot(Xc,Yc,'+');
            plot(Xq,Yq,'ro');
            hold off
        end
        
    end
    
end

