function correctPRtn = robustCameraRecovery(E, pts_1, pts_2)

%init counts
A=0;
B=0;
C=0;
D=0;

% triangulate each point and add to respective case
for i=1:size(pts_1,1)
    
    %points
    q = pts_1(i,:);
    q_prime = pts_2(i,:);
    
    [correctP,CASE] = recoverCameraMat(E, q', q_prime');
    
    %increment based on case
    if(CASE==1)
        A = A + 1;
        AMat = correctP;
    elseif(CASE==2)
        B = B + 1;
        BMat = correctP;
    elseif(CASE==3)
        C = C + 1;
        CMat = correctP;
    else
        D = D + 1;
        DMat = correctP;
    end    
end

%return correctP
maxVal = max([A,B,C,D]);
if(maxVal==A)
    correctPRtn = AMat;
elseif(maxVal==B)
    correctPRtn = BMat;
elseif(maxVal==C)
    correctPRtn = CMat;
else
    correctPRtn = DMat;
end