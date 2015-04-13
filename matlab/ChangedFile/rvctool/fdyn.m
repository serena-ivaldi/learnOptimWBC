%SerialLink.fdyn Integrate forward dynamics
%
% [T,Q,QD] = R.fdyn(T1, TORQFUN) integrates the dynamics of the robot over 
% the time  interval 0 to T and returns vectors of time TI, joint position Q
% and joint velocity QD.  The initial joint position and velocity are zero.
% The torque applied to the joints is computed by the user function TORQFUN:
%
% [TI,Q,QD] = R.fdyn(T, TORQFUN, Q0, QD0) as above but allows the initial
% joint position and velocity to be specified.
%
% The control torque is computed by a user defined function
%
%        TAU = TORQFUN(T, Q, QD, ARG1, ARG2, ...)
%
% where Q and QD are the manipulator joint coordinate and velocity state 
% respectively, and T is the current time. 
%
% [T,Q,QD] = R.fdyn(T1, TORQFUN, Q0, QD0, ARG1, ARG2, ...) allows optional 
% arguments to be passed through to the user function.
%
% Note::
% - This function performs poorly with non-linear joint friction, such as
%   Coulomb friction.  The R.nofriction() method can be used to set this 
%   friction to zero.
% - If TORQFUN is not specified, or is given as 0 or [],  then zero torque
%   is applied to the manipulator joints.
% - The builtin integration function ode45() is used.
%
% See also SerialLink.accel, SerialLink.nofriction, SerialLink.rne, ode45.




% Copyright (C) 1993-2014, by Peter I. Corke
%
% This file is part of The Robotics Toolbox for MATLAB (RTB).
% 
% RTB is free software: you can redistribute it and/or modify
% it under the terms of the GNU Lesser General Public License as published by
% the Free Software Foundation, either version 3 of the License, or
% (at your option) any later version.
% 
% RTB is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU Lesser General Public License for more details.
% 
% You should have received a copy of the GNU Leser General Public License
% along with RTB.  If not, see <http://www.gnu.org/licenses/>.
%
% http://www.petercorke.com

function [t, q, qd] = fdyn(robot, time_struct, object, q0, qd0,fixed_step,varargin)

    % check the Matlab version, since ode45 syntax has changed
    if verLessThan('matlab', '7')  
        error('fdyn now requires Matlab version >= 7');
    end

    time = time_struct.ti:time_struct.step:time_struct.tf;
    
    n = robot.n;
    
    %TO FIX
    if nargin == 2
        torqfun = 0;
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 3
        q0 = zeros(1,n);
        qd0 = zeros(1,n);
    elseif nargin == 4
        qd0 = zeros(1,n);
    end

    % concatenate q and qd into the initial state vector
    q0 = [q0(:); qd0(:)];   
    
    try
        if(fixed_step)
            y = ode4(@fdyn2,time,q0,robot,object,varargin{:});
            t = time';
        else
            [t,y] = ode15s(@fdyn2, [0 time_struct.tf], q0, [], robot, object,varargin{:});
        end      
        q = y(:,1:n);
        qd = y(:,n+1:2*n);
    catch err
        q = y(:,1:n);
        qd = y(:,n+1:2*n); 
        rethrow(err);
    end

end


%FDYN2  private function called by FDYN
%
%   XDD = FDYN2(T, X, FLAG, ROBOT, TORQUEFUN)
%
% Called by FDYN to evaluate the robot velocity and acceleration for
% forward dynamics.  T is the current time, X = [Q QD] is the state vector,
% ROBOT is the object being integrated, and TORQUEFUN is the string name of
% the function to compute joint torques and called as
%
%       TAU = TORQUEFUN(T, X)
%
% if not given zero joint torques are assumed.
%
% The result is XDD = [QD QDD].
function xd = fdyn2(t, x, robot, object, varargin)

    n = robot.n;

    q = x(1:n)';
    qd = x(n+1:2*n)';

    % evaluate the torque function if one is given
    if isa(object, 'function_handle')
        tau = object(robot, t, q, qd, varargin{:});
     elseif isobject(object)
        tau = object.Policy(t,q,qd);
    else   
        tau = zeros(1,n);
    end
    
    %control if tau is not a row vector 
    if(~isrow(tau))
        tau = tau';    
    end
    
    qdd = robot.accel(x(1:n,1)', x(n+1:2*n,1)', tau);
    xd = [x(n+1:2*n,1); qdd];
end
