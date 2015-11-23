function RotationMatrixTest()
    eps = 1e-10;
    % 1. check for evil cases
    CheckNullRotationMatrix(RotationMatrix([0 0 0], 'exponentialMap'))
    CheckNullRotationMatrix(RotationMatrix([0 0 0], 'eulerAngles'))
    CheckNullRotationMatrix(RotationMatrix(eye(3), 'rotationMatrix'))
    CheckNullRotationMatrix(RotationMatrix([0 0 0 1]', 'quaternion'))
    
    %2. diagonal cases
    R = RotationMatrix([1,0,0; 0,-1,0; 0,0,-1], 'rotationMatrix');
    assert(max(abs(R.GetExponentialMap() - [pi;0;0])) < eps)
    eulerAngles = R.GetEulerAngles();
    assert(all(abs(eulerAngles(1,:)-[pi,0,0])<eps) || all(abs(eulerAngles(2,:)-[pi,0,0])<eps))
    assert(all(abs(R.GetQuaternion()-[1;0;0;0])) < eps)
        
    R = RotationMatrix([-1,0,0; 0,1,0; 0,0,-1], 'rotationMatrix');
    assert(max(abs(R.GetExponentialMap() - [0;pi;0])) < eps)
    eulerAngles = R.GetEulerAngles();
    assert(all(abs(eulerAngles(1,:)-[0,pi,0])<eps) || all(abs(eulerAngles(2,:)-[0,pi,0])<eps))
    assert(all(abs(R.GetQuaternion()-[0;1;0;0])) < eps)

    R = RotationMatrix([-1,0 0; 0,-1,0; 0,0,1], 'rotationMatrix');
    assert(max(abs(R.GetExponentialMap() - [0;0;pi])) < eps)
    eulerAngles = R.GetEulerAngles();
    assert(all(abs(eulerAngles(1,:)-[0,0,pi])<eps) || all(abs(eulerAngles(2,:)-[0,0,pi])<eps))
    assert(all(abs(R.GetQuaternion()-[0;0;1;0])) < eps)
    
    
    warning('off','all'); % turn all warnings display off
    for iterations = 1:1000
        % 3. check for gimbal lock
        rotationMatrix = RotationMatrix([rand(), pi/2, rand()], 'eulerAngles');
        eulerAngles = rotationMatrix.GetEulerAngles();
        [msglast, msgidlast] = lastwarn;
        assert(strcmp(msglast, 'GIMBAL LOCK! Infinite solutions are possible. returning one possible solution'))
        R2 = RotationMatrix(eulerAngles(1,:), 'eulerAngles');
        assert(all(all(abs(R2.GetRotationMatrix() - rotationMatrix.GetRotationMatrix()) < eps)))
        R2 = RotationMatrix(eulerAngles(2,:), 'eulerAngles');
        assert(all(all(abs(R2.GetRotationMatrix() - rotationMatrix.GetRotationMatrix()) < eps)))
        
        
        % 4. check that if I create a matrix from something I get the same
        % thing back (or an equivalent rotation matrix)
        eulerAngles = -pi/2 + pi * rand([1,3]);
        rotationMatrix1 = RotationMatrix(eulerAngles, 'eulerAngles');
        eulerAnglesSupposed = rotationMatrix1.GetEulerAngles();
        assert(all(abs(eulerAngles-eulerAnglesSupposed(1,:))<eps) || all(abs(eulerAngles-eulerAnglesSupposed(2,:))<eps))
        
        exponentialMap = -2*pi + 4*pi*rand([3,1]);
        theta = mod(norm(exponentialMap), 2*pi);
        rotationMatrix1 = RotationMatrix(exponentialMap, 'exponentialMap');
        exponentialMapSupposed = rotationMatrix1.GetExponentialMap();
        thetaSupposed = norm(exponentialMapSupposed);
        assert(all(abs(exponentialMap - exponentialMapSupposed)<eps) || (theta - 2*pi - thetaSupposed)<eps)
        
        quaternion = rand([4,1]);
        rotationMatrix1 = RotationMatrix(quaternion, 'quaternion');
        quaternionSupposed = rotationMatrix1.GetQuaternion();
        rotationMatrix2 = RotationMatrix(quaternionSupposed, 'quaternion');
        assert(all(all(abs(rotationMatrix2.GetRotationMatrix()-rotationMatrix1.GetRotationMatrix())<eps)))
        
        
        % 5. general cross- consistency checks
        exponentialMap = - 10 + 20*rand([1,3]);
        r1 = RotationMatrix(exponentialMap, 'exponentialMap');
        euler1 = r1.GetEulerAngles();
        r2 = RotationMatrix(euler1(1,:), 'eulerAngles');
        euler2 = r2.GetEulerAngles();
        assert(all(abs(euler1(:) - euler2(:))<eps))
        r2 = RotationMatrix(euler1(2,:), 'eulerAngles');
        euler2 = r2.GetEulerAngles();
        assert(all(abs(euler1(:) - euler2(:))<eps))
        
        eulerAngles = -10 + 20*rand([1,3]);
        r1 = RotationMatrix(eulerAngles, 'eulerAngles');
        r2 = RotationMatrix(r1.GetExponentialMap(), 'exponentialMap');
        assert(all(all(abs(r1.GetEulerAngles() - r2.GetEulerAngles())<1e-5)))
        assert(all(abs(r1.GetExponentialMap() - r2.GetExponentialMap())<1e-5))
        
        quaternion = rand([4,1]);
        quaternion = quaternion/norm(quaternion);
        r1 = RotationMatrix(quaternion, 'quaternion');
        euler1 = r1.GetEulerAngles();
        r2 = RotationMatrix(euler1(1,:), 'eulerAngles');
        assert(all(all(abs(r1.GetEulerAngles() - r2.GetEulerAngles())<eps)))
        r2 = RotationMatrix(euler1(2,:), 'eulerAngles');
        assert(all(all(abs(r1.GetEulerAngles() - r2.GetEulerAngles())<eps)))
    end
    warning('on','all'); % turn all warnings display on
end

function CheckNullRotationMatrix(nullRotationMatrix)
    identityMatrix = eye(3);
    nullR = nullRotationMatrix.GetRotationMatrix();
    assert(all(nullR(:) == identityMatrix(:)))
    nullEuler = nullRotationMatrix.GetEulerAngles();
    assert(all(nullEuler(1,:) == 0) || all(nullEuler(2,:) == 0))
    nullExp = nullRotationMatrix.GetExponentialMap();
    assert(all(nullExp(1,:) == 0))
    nullQuaternion = nullRotationMatrix.GetQuaternion();
    nullExpectedQuaternion = [0 0 0 1]';
    assert(all(nullQuaternion == nullExpectedQuaternion))
end