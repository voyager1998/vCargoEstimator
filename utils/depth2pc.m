% This function is not used for this project because ToF camera doesn't
% return z value, instead, it returns the distance d = sqrt(x^2+y^2+z^2).
% Thus, we use tof2pc.
function ptcloud = depth2pc(D, C)
s = size(D);
w = s(1);
h = s(2);
ptcloud = zeros(s);
for i = 1: w
    for j = 1: h
        if D(i,j) > 1000
            continue;
        end
        proj = [i*double(D(i,j)); j*double(D(i,j)); double(D(i,j))];
        pos = C\proj;
        ptcloud(i,j,1) = pos(1);
        ptcloud(i,j,2) = pos(2);
        ptcloud(i,j,3) = pos(3);
    end
end
end