function B = computeB(A)

%define indices -- i replaced with n
eIndex = 5;
fIndex = 6;
gIndex = 7;
hIndex = 8;
nIndex = 9;
jIndex = 10;

%define basis
syms x;
syms y;
syms z;
norm_basis = [x^3, y^3, x^2*y, x*y^2, x^2*z, x^2, y^2*z, y^2, x*y*z, x*y, x*z^2, x*z, x, y*z^2, y*z, y, z^3, z^2, z, 1];
norm_basis = transpose(norm_basis);

%recreate current equations
e = A(eIndex,:)*norm_basis;
f = A(fIndex,:)*norm_basis;
g = A(gIndex,:)*norm_basis;
h = A(hIndex,:)*norm_basis;
n = A(nIndex,:)*norm_basis;
j = A(jIndex,:)*norm_basis;

%define additional equations
k = simplify(e - z*f);
l = simplify(g - z*h);
m = simplify(n - z*j);

%create B
B = sym(zeros(3,3));
B(1,:) = computeBRow(k);
B(2,:) = computeBRow(l);
B(3,:) = computeBRow(m);