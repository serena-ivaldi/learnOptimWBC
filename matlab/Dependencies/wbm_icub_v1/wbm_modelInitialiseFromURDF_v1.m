function [  ] = wbm_modelInitialiseFromURDF_v1( varargin )
%WMB_MODELINITIALISE initialises the whole body model using an URDF file
%   Arguments :
%       path to the URDF file to load as a string,
%   Returns :   None
%
% Author : Silvio Traversaro (silvio.traversaro@iit.it)
% Genova, Nov 2015

    switch(nargin)
        case 1
            wholeBodyModel('model-initialise-urdf',varargin{1});
        otherwise
            disp('modelInitialiseFromURDF : Incorrect number of arguments, check docs');
    end
end
