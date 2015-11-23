function ARow = computeARow(constraint, NORM_coeffs)
syms x;
syms y;
syms z;

%use coeffs to give us its ordering
ARowCoeff = coeffs(constraint);

%reorder the coefficients in the ordering we want
ARow = zeros(20,1);
for i=1:20
    ARow(NORM_coeffs(i)) = ARowCoeff(i);
end
