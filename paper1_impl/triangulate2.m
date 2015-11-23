function Q = triangulate2(E, q, q_prime, P)

%compute a,b,c,d
diag = [1 0 0; 0 1 0; 0 0 0];
a = E'*q_prime;
b = cross(q, diag*a);
c = cross(q_prime, diag*E*q);
d = cross(a,b);

%compute A, B, C
A = [a' 0]';
B = [b' 0]';
C = P'*c;

%compute Q
Q= [d'*C(4), -1*(d(1)*C(1) + d(2)*C(2) + d(3)*C(3))]';