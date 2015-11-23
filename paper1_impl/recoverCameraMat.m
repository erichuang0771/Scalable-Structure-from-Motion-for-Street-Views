function [correctP, CASE] = recoverCameraMat(E, q, q_prime)

%preliminaries
[U,S,V] = svd(E);
D = [0 1 0; -1 0 0; 0 0 1];
t_u = [U(1,3), U(2,3), U(3,3)]';
R_a = U*D*V';
R_b = U*D'*V';

%compute P's
P_A = [R_a t_u];
P_B = [R_a -1*t_u];
P_C = [R_b t_u];
P_D = [R_b -1*t_u];

%cheirality constraint
Q = triangulate2(E, q, q_prime, P_A);
c1 = Q(3)*Q(4);
PAQ = P_A*Q;
c2 = PAQ(3)*Q(4);

%return based on cases
if(c1 > 0 && c2 > 0)
    correctP = P_A;
    CASE = 1;
elseif(c1 < 0 && c2 < 0)
    correctP = P_B;
    CASE = 2;
elseif(c1*c2 < 0)
    correctP = P_C;
    CASE = 3;
else
    correctP = P_D;
    CASE = 4;
end
