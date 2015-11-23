function M2 = camera2(E, K2)

[U,SIG,V] = svd(E);

% S = [t]_x,  t = u3
u3 = U(:,3);
S = [0 -u3(3) u3(2) ; u3(3) 0 -u3(1) ; -u3(2) u3(1) 0];

% Recover Z
% ------------
%                          S = U*Z*U'
%                 inv(U) * S = Z*U'
%              (inv(U) * S)' = U*Z'
%  (inv(U) * (inv(U) * S)')' = Z
%Z = (inv(U) * (inv(U) * S)')';
Z = (U \(U \ S)')';

% Can compute W given knowledge of structure of Z
% -----------
% E = alpha*U*SIG*V' = SR = (U*Z*U')(U*W*V') = U*(Z*W)*V'
% SIG = Z*W
% (SIG = [1 0 0 ; 0 1 0 ; 0 0 0])
W = [0 -1 0 ; 1 0 0 ; 0 0 1];

%           R      t
M2_1 = K2*[U*W*V', u3];
M2_2 = K2*[U*W*V', -u3];
M2_3 = K2*[U*W'*V', u3];
M2_4 = K2*[U*W'*V', -u3];

% works for temple
% M2 = M2_3;

M2 = cell(4,1);

M2{1} = M2_1;
M2{2} = M2_2;
M2{3} = M2_3;
M2{4} = M2_4;

end