function BRow = computeBRow(eq)

%define syms
syms x;
syms y;
syms z;

%extract coeffs and terms
[xCoeff, xTerm] = coeffs(eq,x);
[yCoeff, yTerm] = coeffs(eq,y);

%extract x,y equations
xEq = xCoeff(1);
yEq = yCoeff(1);

%extract z equation
remainderX = xCoeff(2);
[y2Coeff, y2Term] = coeffs(remainderX,y);
zEq = y2Coeff(2);

%return BRow
BRow = [xEq, yEq, zEq];