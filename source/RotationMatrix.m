classdef RotationMatrix
    properties
        matrix
    end
    methods (Access = public)
        function obj = RotationMatrix(dataArray, dataType)
            switch dataType
                case 'eulerAngles'
                    obj.matrix = RotationMatrix.ComputeRotationMatrixFromEulerAngles(dataArray(1), dataArray(2), dataArray(3));
                case 'exponentialMap'
                    obj.matrix = RotationMatrix.ComputeRotationMatrixFromExponentialMap(dataArray(1:3));
                    obj.matrix = obj.matrix(1:3, 1:3);
                case 'rotationMatrix'
                    RotationMatrix.AssertIsARotationMatrix(dataArray);
                    obj.matrix = dataArray;
                case 'quaternion'
                    obj.matrix = RotationMatrix.ComputeRotationMatrixFromQuaternion(dataArray);
                otherwise
                    error('dataType must be either eulerAngles, exponentialMap or rotationMatrix')
            end
        end
        
        function rotationArray = GetExponentialMap(obj)
            [theta, axis] = RotationMatrix.GetEulerAxis(obj.matrix);
            rotationArray = theta*axis;
            if any(isnan((rotationArray))) == 1
                rotationArray = [0; 0; 0];
            end
        end
        
        function eulerAngles = GetEulerAngles(obj)
            % eulerAngles = [2 x 3] array; each row contains a triplet of
            % possible euler angles.
            % for further details: http://www.soi.city.ac.uk/~sbbh653/publications/euler.pdf
            
            eulerAngles = zeros(2, 3); % if no gimbal lock is detected, we return 2 possible solutions
            
            if (abs(abs(obj.matrix(3,1)) - 1) < 1e-12)
                warning('GIMBAL LOCK! Infinite solutions are possible. returning one possible solution')
                if (abs(obj.matrix(3,1) - 1) < 1e-12) % R31 == 1
                    eulerAngles(:,1) = atan2(-obj.matrix(1,2), -obj.matrix(1,3));
                    eulerAngles(:, 2) = -pi/2;
                else
                    eulerAngles(:,1) = atan2(obj.matrix(1,2), obj.matrix(1,3));
                    eulerAngles(:, 2) = pi/2;
                end
                return;
            end
            
            theta = -asin(obj.matrix(3,1));
            theta2 = pi - theta;
            % psi1, psi2
            eulerAngles(1,1) = atan2(obj.matrix(3,2)/cos(theta), obj.matrix(3,3)/cos(theta));
            eulerAngles(2,1) = atan2(obj.matrix(3,2)/cos(theta2), obj.matrix(3,3)/cos(theta2));
            
            % theta1, theta2
            eulerAngles(1,2) = theta;
            eulerAngles(2,2) = theta2;
            
            %phi1, phi2
            eulerAngles(1,3) = atan2(obj.matrix(2,1)/cos(theta), obj.matrix(1,1)/cos(theta));
            eulerAngles(2,3) = atan2(obj.matrix(2,1)/cos(theta2), obj.matrix(1,1)/cos(theta2));
        end
        
        function rotationMatrix = GetRotationMatrix(obj)
            rotationMatrix = obj.matrix;
        end
        
        function quaternion = GetQuaternion(obj)
            % for further details:  http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
            quaternion = zeros(4, 1);
            tr = trace(obj.matrix);
            if (tr > 1e-2)
                temp1 = sqrt(1 + tr)*2;
                quaternion(4) = temp1/4;
                quaternion(1) = (obj.matrix(3, 2) - obj.matrix(2, 3))/temp1;
                quaternion(2) = (obj.matrix(1, 3) - obj.matrix(3, 1))/temp1;
                quaternion(3) = (obj.matrix(2, 1) - obj.matrix(1, 2))/temp1;
            else
                if ((obj.matrix(1, 1) > obj.matrix(2, 2)) && (obj.matrix(1, 1) > obj.matrix(3, 3)))
                    temp1 = sqrt(1 + obj.matrix(1, 1) - obj.matrix(2, 2) - obj.matrix(3, 3)) * 2;
                    quaternion(1) = temp1/4;
                    quaternion(2) = (obj.matrix(1, 2) + obj.matrix(2, 1)) / temp1;
                    quaternion(3) = (obj.matrix(1, 3) + obj.matrix(3, 1)) / temp1;
                    quaternion(4) = (obj.matrix(3, 2) - obj.matrix(2, 3)) / temp1;
                elseif (obj.matrix(2, 2) > obj.matrix(3, 3))
                    temp1 = sqrt(1 + obj.matrix(2, 2) - obj.matrix(1, 1) - obj.matrix(3, 3)) * 2;
                    quaternion(1) = (obj.matrix(1, 2) + obj.matrix(2, 1)) / temp1;
                    quaternion(2) = temp1/4;
                    quaternion(3) = (obj.matrix(3, 2) + obj.matrix(2, 3)) / temp1;
                    quaternion(4) = (obj.matrix(1, 3) - obj.matrix(3, 1)) / temp1;
                else
                    temp1 = sqrt(1 + obj.matrix(3, 3) - obj.matrix(1, 1) - obj.matrix(2, 2)) * 2;
                    quaternion(1) = (obj.matrix(1, 3) + obj.matrix(3, 1)) / temp1;
                    quaternion(2) = (obj.matrix(3, 2) + obj.matrix(2, 3)) / temp1;
                    quaternion(3) = 0.25 * temp1;
                    quaternion(4) = (obj.matrix(2, 1) - obj.matrix(1, 2)) / temp1;
                end
            end
        end
    end
    
    methods(Static, Access = public)
        function rotationMatrix = ComputeRotationMatrixFromQuaternion(dataArray)
            dataArray = dataArray./norm(dataArray);
            q1 = dataArray(1);
            q2 = dataArray(2);
            q3 = dataArray(3);
            q4 = dataArray(4);
            
            rotationMatrix = [1-2*q2*q2-2*q3*q3,  2*(q1*q2-q3*q4),    2*(q1*q3+q2*q4)
                2*(q1*q2+q3*q4),    1-2*q1*q1-2*q3*q3,  2*(q2*q3-q1*q4)
                2*(q1*q3-q2*q4),    2*(q1*q4+q2*q3),    1-2*q1*q1-2*q2*q2];
        end
        
        function rotationMatrix = ComputeRotationMatrixFromEulerAngles(alpha, beta, gamma)
            % inputs are the intrinsic rotation angles in RADIANTS
            Rx = [1 0 0; 0 cos(alpha) -sin(alpha); 0 sin(alpha) cos(alpha)];
            Ry = [cos(beta) 0 sin(beta); 0 1 0; -sin(beta) 0 cos(beta)];
            Rz = [cos(gamma) -sin(gamma) 0; sin(gamma) cos(gamma) 0; 0    0   1];
            rotationMatrix = Rz*Ry*Rx;
        end
        
        function rotationMatrix = ComputeRotationMatrixFromExponentialMap(exponentialMap)
            
            if all(exponentialMap == 0) == 1
                rotationMatrix = eye(3);
                return
            end
            
            exponentialMap = RotationMatrix.CheckExponentialMapParametrization(exponentialMap);
            theta = norm(exponentialMap);
            
            exponentialMap = exponentialMap/theta;
            
            x = exponentialMap(1);
            y = exponentialMap(2);
            z = exponentialMap(3);
            
            s = sin(theta/2);
            c = cos(theta/2);
            rotationMatrix = [2*(x^2-1)*s^2+1  2*x*y*s^2-2*z*c*s  2*x*z*s^2+2*y*c*s
                2*x*y*s^2+2*z*c*s  2*(y^2-1)*s^2+1  2*y*z*s^2-2*x*c*s
                2*x*z*s^2-2*y*c*s 2*y*z*s^2+2*x*c*s  2*(z^2-1)*s^2+1];
        end
        
        function expMap = CheckExponentialMapParametrization(expMap)
            theta = norm(expMap);
            if theta > 2*pi
                theta2 = theta;
                theta = mod(theta,2*pi);
                expMap = expMap*theta/theta2;
            end
            if theta > pi
                expMap = expMap*(1 - 2*pi/theta);
            end
        end
    end
    
    methods(Static, Access = private)
        function [theta, axis] = GetEulerAxis(matrix)
            RotationMatrix.AssertIsARotationMatrix(matrix);
            
            [V, d] = eig(matrix);
            [~, index] = min(abs(diag(d) -1));
            axis = V(:,index);
            
            cosTheta = (trace(matrix) - 1)/2;
            [~, index] = max(abs(axis));
            switch(index)
                case 1
                    sinTheta = (matrix(3,2) - matrix(2,3))/(2*axis(index));
                case 2
                    sinTheta = (matrix(1,3) - matrix(3,1))/(2*axis(index));
                case 3
                    sinTheta = (matrix(2,1) - matrix(1,2))/(2*axis(index));
            end
            theta = atan2(sinTheta, cosTheta);
        end
        
        function AssertIsARotationMatrix(dataArray)
            tempInv = inv(dataArray);
            tempTransl = dataArray';
            assert(max(abs(tempInv(:) - tempTransl(:) )) < 1e-6)
            assert(abs(det(dataArray)-1) < 1e-6)
        end
    end
end
