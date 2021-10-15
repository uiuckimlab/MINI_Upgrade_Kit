function T = DHparameter(DH)
T = zeros(4,4,size(DH,1));
for j=1:size(DH,1)
    T(:,:,j) = [cos(DH(j,4)) -sin(DH(j,4)) 0 DH(j,1);
            sin(DH(j,4))*cos(DH(j,2)) cos(DH(j,4))*cos(DH(j,2)) -sin(DH(j,2)) -sin(DH(j,2))*DH(j,3);
            sin(DH(j,4))*sin(DH(j,2)) cos(DH(j,4))*sin(DH(j,2)) cos(DH(j,2))  cos(DH(j,2))*DH(j,3);
            0 0 0 1];
end

