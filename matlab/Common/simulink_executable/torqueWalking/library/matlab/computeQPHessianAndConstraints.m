% COMPUTEQPHESSIANANDCONSTRAINTS computes Hessian matrix, gradient and
%                                constraints for integration based inverse 
%                                kinematics.
%
% IMPLEMENTATION: we assume to have a desired set of joint accelerations
%                 sDDot_star. We also assume to have a desired set of
%                 accelerations for a Cartesian task, namely acc_task_star.
%
% USAGE: please note that this function has been designed for being inserted 
%        in a Simulink model.
%
% FORMAT: [Hessian, gradient, ConstraintMatrix, biasVectorConstraint] = ...
%             computeQPHessianAndConstraints(s, nu, s_sDot_sDDot_ref, ...
%                                            impedances, dampings, J, JDot_nu, ...
%                                            acc_task_star) 
%
% INPUT:  - s = [ROBOT_DOF * 1]joint positions
%         - nu = [ROBOT_DOF + 6 * 1] state velocities
%         - s_sDot_sDDot_ref = [ROBOT_DOF * 3] joint unconstrained references
%         - impedances = [ROBOT_DOF * ROBOT_DOF] joint position gains
%         - dampings = [ROBOT_DOF * ROBOT_DOF] joint velocity gains
%         - J = [n_c * ROBOT_DOF + 6] task Jacobian
%         - JDot_nu = [n_c * 1] task Jacobian derivative times state velocities
%         - acc_task_star = [n_c * 1] task desired accelerations 
%
% OUTPUT: - Hessian = [ROBOT_DOF + 6 * ROBOT_DOF + 6] Hessian matrix 
%         - gradient = [ROBOT_DOF + 6 * 1] gradient for QP optimization
%         - ConstraintMatrix = [ n_c * ROBOT_DOF + 6] constraint matrix
%         - biasVectorConstraint = [ n_c * 1] bias vector constraints
%
% Authors: Daniele Pucci, Marie Charbonneau, Gabriele Nava
%          
%          all authors are with the Italian Istitute of Technology (IIT)
%          email: name.surname@iit.it
%
% Genoa, Dec 2017
%

%% --- Initialization ---
function [Hessian, gradient, ConstraintMatrix, biasVectorConstraint] = ...
             computeQPHessianAndConstraints(s, nu, s_sDot_sDDot_ref, impedances, dampings, J, JDot_nu, acc_task_star, feetInContact, Sat, Config)
      
     % get joint velocities    
     sDot    = nu(7:end);
         
     % compute desired joint acceleration for postural task
     sDDot_star =  s_sDot_sDDot_ref(:,3) ...
                  -diag(impedances)*(s -s_sDot_sDDot_ref(:,1)) ...
                  -diag(dampings)*(sDot -s_sDot_sDDot_ref(:,2));                      

     % to ensure the convergence of the base accelerations to zero, a feedback
     % term on base velocities is considered. the parameter k is a positive gain.
     k          = 1;
     
     if Config.QP_USE_STRICT_TASK_PRIORITIES 
         % In the case of strict task priorities,
         % the objective is to find a set of state accelerations nuDot 
         % which minimize the error (sDDot-sDDot_star), while subject 
         % to the constraint that the task desired accelerations must 
         % be achieved. The optimization procedure can be written as:
         %
         %           nuDot = [nuDot_b; sDDot];
         %
         %           nuDot = min(1/2*|(sDDot-sDDot_star) + nuDot_b|^2)
         %
         %              s.t. J*nuDot + JDot_nu = acc_task_star   
         % or
         %           nuDot* = argmin 1/2 | nuDot - nuDot_ref |^2  (1)
         %              s.t.  J * nuDot = acc_task_ref - JDot_nu  (2)
         %
         % (1) may be reformulated in the form of a QP:
         % nuDot* = argmin 1/2 (nuDot' * 1 * nuDot) + (nuDot' * nuDot_ref)
         % 
         % nuDot_ref is defined using a combination of feedback terms on
         % base velocity and joint accelerations:
         % nuDot_ref = [k*nu(1:6); -sDDot_star]
         %
         % Only CoM, orientation, l_foot and r_foot tasks are considered.
         % Otherwise, more constraints than DOF may make the QP unfeasible
         
         % compute Hessian matrix
         Hessian = eye(size(s,1) +6);
     
         % compute gradient
         gradient   = [k*nu(1:6); -sDDot_star];
         
         % compute equality constraint matrix
         ConstraintMatrix = J(1:18,:);
         
         % compute bias vector constraints
         biasVectorConstraint = acc_task_star(1:18) -JDot_nu(1:18);
         
         %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %% TEMPORARY FIX FOR SOLVING A BUG IN QPOAES WITH EQUALITY CONSTRAINTS
         %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         correction = 1e-9.*size(3, size(s,1) +6);
         ConstraintMatrix(end-2:end,:) = ConstraintMatrix(end-2:end,:) -correction;
         %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         %% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
         
     else  %use soft tasks priorities
         % In the case of soft task priorities, the inverse kinematics
         % problem is formulated as
         %
         % nuDot* = argmin 1/2 * Sum_over_i [w_i | nuDot - nuDot_ref_i |^2]  (1)
         %
         % where i refers to each task considered (postural, CoM, orientation task, feet, hands).
         %
         % For example, for the postural task (nuDot_ref is defined using a combination of feedback terms on
         % base velocity and joint accelerations):
         % w_i = weightPostural
         % nuDot_ref_i = [k*nu(1:6); -sDDot_star]
         %
         % Or for a cartesian task:
         % w_i = weight_cartesian
         % nuDot_ref_i = pinv(J_cartesian * (JDot_nu_cartesian - acc_task_star_cartesian)
         %
         % (1) may be reformulated in the form of a QP:
         % nuDot* = argmin 1/2 (nuDot' * Sum_over_i [w_i] * nuDot) + Sum_over_i [w_i * (nuDot' * nuDot_ref_i)]
         %         
         
         % adjust weights for the feet tasks depending on whether the foot
         % is in contact with the ground or not
         if feetInContact(1) > 0.1
             weightLeftFoot  = Sat.weightStanceFoot;
         else
             weightLeftFoot  = Sat.weightSwingFoot;
         end
         if feetInContact(2) > 0.1
             weightRightFoot  = Sat.weightStanceFoot;
         else
             weightRightFoot  = Sat.weightSwingFoot;
         end
         
         % sum of task weights for the Hessian
         sum_task_weights = Sat.weightPostural + Sat.weightCoM  + Sat.weightRotTask + weightLeftFoot + weightRightFoot ...
             + Sat.weightLeftHand + Sat.weightRightHand;
         
         % compute Hessian matrix
         Hessian = eye(size(nu)) * sum_task_weights;
         
         % compute gradient
         gradient =  Sat.weightPostural  * [k*nu(1:6); -sDDot_star] ...
                   + Sat.weightCoM       * pinv(J( 1: 3, :)) * (JDot_nu( 1: 3) - acc_task_star( 1: 3)) ...
                   + Sat.weightRotTask   * pinv(J( 4: 6, :)) * (JDot_nu( 4: 6) - acc_task_star( 4: 6)) ...
                   + weightLeftFoot      * pinv(J( 7:12, :)) * (JDot_nu( 7:12) - acc_task_star( 7:12)) ...
                   + weightRightFoot     * pinv(J(13:18, :)) * (JDot_nu(13:18) - acc_task_star(13:18)) ...
                   + Sat.weightLeftHand  * pinv(J(19:24, :)) * (JDot_nu(19:24) - acc_task_star(19:24)) ...
                   + Sat.weightRightHand * pinv(J(25:30, :)) * (JDot_nu(25:30) - acc_task_star(25:30));
         
         % equality constraint matrix is null
         ConstraintMatrix =  zeros(size(J(1:18,:)));
         
         % bias vector for constraints is null
         biasVectorConstraint = zeros(size(acc_task_star(1:18))); 
     end
     
end