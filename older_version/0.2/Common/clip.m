function [xclp,varargout] = clip(x,xmin,xmax,varargin)
% clip(x,xmin,xmax,varargin) returns a clipped or trimmed array containing
% only the values of x between xmin and xmax, inclusive.  Inclusion of any 
% other arrays as inputs returns corresponding clipped arrays. The optional
% argument 'exclusive' can be included to return only values between but
% not including xmin and xmax. 
% 
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% INPUTS AND OUTPUTS:
%
% xclp = clip(x,xmin,xmax) returns an array of all values in x from xmin to
%           xmax, inclusive.
%
% xclp = clip(x,xmin,xmax,'exclusive') returns an array of all values in x 
%           from xmin to xmax, exclusive. 
% 
% [xclp,ind] = clip(x,xmin,xmax) returns an array of all values in x from xmin 
%           to xmax, and also returns the indices of x corresponding to
%           xclp.  Note that xclp = x(ind). 
%   
% [xclp,yclp] = clip(x,xmin,xmax,y) returns xclp and corresponding clipped
%           values of the array y, which should be the same length as x. 
% 
% [xclp,y1clp,y2clp,...,ynclp] = clip(x,xmin,xmax,y1,y2,...yn) returns
%           clipped arrays of x and y1 through yn.  
% 
% [xclp,y1clp,y2clp,...,ynclp,ind] = clip(x,xmin,xmax,y1,y2,...yn) returns
%           clipped arrays of x and y1 through yn and corresponding
%           indices ind. xclp = x(ind) and y1clp = y1(ind) and so on.
% 
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% EXAMPLE:
% 
% % Define raw data: 
% x = (0:.1:100);
% y1 = x.^1.6;
% y2 = x.^1.5-x;
% y3 = 10*x;
% y4 = 100*sin(x/10)+5*x; 
% 
% % Plot raw data: 
% plot(x,y1,'r',x,y2,'b',x,y3,'m',x,y4,'k','linewidth',1); hold on;
% 
% % Clip data to x values between 70 and 90: 
% [xc,y1c,y2c,y3c,y4c,ind] = clip(x,70,90,y1,y2,y3,y4);% <--HERE'S THE FUNCTION
% 
% % Plot clipped data atop raw data: 
% plot(xc,y1c,'r',xc,y2c,'b',xc,y3c,'m',xc,y4c,'k','linewidth',4); 
% 
% % A little housekeeping for the plot: 
% legend('y1','y2','y3','y4','y1_{clipped}','y2_{clipped}','y3_{clipped}',...
%     'y4_{clipped}','location','northwest')
% legend boxoff
% box off
% axis tight
%
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
% Created by Chad A. Greene, August 2013. 
% * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *


% By default, >= and <= are used unless the 'exclusive' tag is declared, in
% which case > and < are used.  Check exclusive/inclusive declaration: 
arginvals = 1:length(varargin);
inclusive = true; 
textin = false; 
for k = 1:length(varargin)
    if strcmpi(varargin{k},'exclusive')||strcmpi(varargin{k},'exclude')
        inclusive = false; 
        textin = true; % indeed, text is an input argument
        textn = k; % and this is the index of the text varargin
    elseif strcmpi(varargin{k},'inclusive')||strcmpi(varargin{k},'include')
        textn = k; % Inclusive is assumed by default. 
        textin = true; 
    end
end
if textin
    arginvals(textn)=[]; 
end

% Find indices of x values between xmin and xmax:
% if the index is one it means that a violations took place so i need to
% clip that dimension
if inclusive
    lo_ind = x <= xmin; 
    up_ind = x >= xmax;
else
    lo_ind = x < xmin;  
    up_ind = x > xmax; 
end

x(lo_ind) = xmin(lo_ind);
x(up_ind) = xmax(up_ind);
xclp = x;
for k = arginvals
    if length(varargin{k})~=length(x)
        fprintf('Length of x does not match length of other input array(s).\n')
        fprintf('Attempting to go spit out some numbers anyway. Verify their integrity.\n')
    end
        varargout{k} = varargin{k}(n); 
end

varargout{k+1} = [lo_ind;up_ind]; % returns indices