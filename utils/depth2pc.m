function ptcloud = depth2pc(D, C)
s = size(D);
w = s(1);
h = s(2);
% ptcloud = zeros(s);
counter = 1;
for i = 1: w
    for j = 1: h
        if D(i,j) > 1000
            continue;
        else
            counter = counter + 1;
        end
    end
end
ptcloud = zeros([counter 3]);
idx = 1;
for i = 1: w
    for j = 1: h
        if D(i,j) > 1000
            continue;
        end
        proj = [i*double(D(i,j)); j*double(D(i,j)); double(D(i,j))];
        proj = cast(proj,'double');
        pos = C\proj;
        ptcloud(idx,1) = pos(1);
        ptcloud(idx,2) = pos(2);
        ptcloud(idx,3) = pos(3);
        idx = idx + 1;
    end
end
end