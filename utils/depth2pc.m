function ptcloud = depth2pc(D, C)
s = size(D);
w = s(1);
h = s(2);
ptcloud = zeros(s);
% invC = inv(C);
for i = 1: w
    for j = 1: h
        proj = [i*D(i,j); j*D(i,j); D(i,j)];
        proj = cast(proj,'double');
        pos = C\proj;
        ptcloud(i,j,1) = pos(1);
        ptcloud(i,j,2) = pos(2);
        ptcloud(i,j,3) = pos(3);
    end
end
end