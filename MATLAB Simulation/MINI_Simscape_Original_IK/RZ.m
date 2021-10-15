function [RZ] = RZ(a)
RZ = [cos(a), -sin(a), 0, 0;
    sin(a), cos(a), 0, 0;
    0, 0, 1, 0;
    0, 0, 0, 1];
end