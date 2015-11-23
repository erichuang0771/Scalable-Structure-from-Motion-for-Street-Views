function [EList, numE] = five_point(points1, points2)

% Note: Points need to be normalized with camera intrinsics (K1 and K2)
% before being passed to algorithm. They also need to be in homogeneous
% coordinates.

% points2' * E * points1 = 0

%size calc
%points_size = size(points1_orig);
%N = points_size(1);
N=5;

% NOTES: Normalizing makes sense but it makes the results look worse. Maybe
% there is a project with this function. Remember to de-normalize the
% points at the end.
% Normalize the points to fix numerical stability issues. This function
% also converts points to homogenous coordinates
%[points1, T1] = normalize_points(points1_orig);
%[points2, T2] = normalize_points(points2_orig);

%compute Q
Q = zeros(N,9);
for i=1:N
    
    %extract params
    q1 = points1(i,1);
    q2 = points1(i,2);
    q3 = points1(i,3);
    q1_prime = points2(i,1);
    q2_prime = points2(i,2);
    q3_prime = points2(i,3);
    
    %plug row into Q
    Q(i,:) = [q1*q1_prime, q2*q1_prime, q3*q1_prime, q1*q2_prime, q2*q2_prime, q3*q2_prime, q1*q3_prime, q2*q3_prime, q3*q3_prime];
    
end

%Step 1 - extract nullspace using SVD
[U,S,V] = svd(Q);

% I think we want to use end rather than lastIndex to find the right null
% space - Andrew 11/21/2010
X = V(:, end-3);
Y = V(:, end-2);
Z = V(:, end-1);
W = V(:, end);

%rehape X,Y,Z,W to 3x3
X = reshapeMatrix(X);
Y = reshapeMatrix(Y);
Z = reshapeMatrix(Z);
W = reshapeMatrix(W);

%create E
syms x;
syms y;
syms z;
w=1;
E = x*X + y*Y + z*Z + w*W;

%step 2 - expansion of cubic constraints

%we will store the 10 constraints in a cell arr called constraints. We'll
%call the index of that cell array constraints_i.
constraints_i = 1;

%compute det(E) constraint and put in constraints cell arr
det_E =  o2(o1(E(1,2),E(2,3)) - o1(E(1,3),E(2,2)),E(3,1)) + o2(o1(E(1,3),E(2,1)) - o1(E(1,1),E(2,3)),E(3,2)) + o2(o1(E(1,1),E(2,2)) - o1(E(1,2),E(2,1)),E(3,3));
constraints = cell(1,10);
constraints{constraints_i} = det_E;
constraints_i = constraints_i + 1;

%compute EE^T
EE_t = sym(zeros(3,3));
for i=1:3
    for j=1:3
        for k=1:3
            EE_t(i,j) = EE_t(i,j) + o1(E(i,k), E(j,k));
        end
    end
end

%compute lambda
lambda = EE_t - 1/2*trace(EE_t)*eye(3);

%compute lambda*E
lambdaE = sym(zeros(3,3));
for i=1:3
    for j=1:3
        for k=1:3
            lambdaE(i,j) = lambdaE(i,j)  + o2(lambda(i,k),E(k,j));
        end
    end
end

%put lambda E constraints into constraints cell array
for i=1:3
    for j=1:3
        constraints{constraints_i} = lambdaE(i,j);
        constraints_i = constraints_i + 1;
    end
end
numConstraints = constraints_i -1;

%compute A
A = zeros(numConstraints,20);
%compute the ordering of coefficients that we want coeffs function to give
%ideally give us (though it will the coefficients h_pts2in the order that its designed to give us).
NORM = x^3 + 2*y^3 + 3*x^2*y + 4*x*y^2 + 5*x^2*z + 6*x^2 + 7*y^2*z + 8*y^2 + 9*x*y*z + 10*x*y + 11*x*z^2 + 12*x*z + 13*x + 14*y*z^2 + 15*y*z + 16*y + 17*z^3 + 18*z^2 + 19*z + 20;
NORM_coeffs = double(coeffs(NORM));
for i=1:numConstraints
    A(i,:) = computeARow(constraints{i}, NORM_coeffs);
end

%step 3 - Gauss-Jordan Elimination with Partial Pivoting on A (and compute
%B from that).
A = rref(A);
B=computeB(A);

%step 4 - Determinant expansion
p1 = B(1,2)*B(2,3) - B(1,3)*B(2,2);
p2 = B(1,3)*B(2,1) - B(1,1)*B(2,3);
p3 = B(1,1)*B(2,2) - B(1,2)*B(2,1);
n = p1*B(3,1) + p2*B(3,2) + p3*B(3,3);
n= simplify(n);

%step 5 - root extraction
polyCoeffs = sym2poly(n);
rootsList = roots(polyCoeffs);
rootsList_size = size(rootsList);
numRoots =  rootsList_size(1);

%recover an essential matrix for each real root (i.e. discard imaginary
%roots) and store in our EList cell array structure that the function returns.
EList_i=1;
for i=1:numRoots
    
    currentRoot = rootsList(i);
    if(currentRoot==real(currentRoot))
        
        %extract E params
        z_rtn = currentRoot;
        x_rtn = subs(p1, 'z', z_rtn) / subs(p3, 'z', z_rtn);
        y_rtn = subs(p2, 'z', z_rtn) / subs(p3, 'z', z_rtn);
        w_rtn = 1;
        
        %compute E using params
        E_rtn = x_rtn*X + y_rtn*Y + z_rtn*Z + w_rtn*W;
        
        % Un-normalize and store in EList
        %EList{EList_i} = T2' * E_rtn * T1;
        EList{EList_i} = E_rtn;
        EList_i = EList_i + 1;
    end
end
numE = EList_i - 1;