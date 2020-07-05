% solve Aw = b

function corner = findIntersection(plane1, plane2, plane3)
    A = [plane1(1:3); plane2(1:3); plane3(1:3)];
    b = [-plane1(4); -plane2(4); -plane3(4)]; % notice the minus sign
    w = (A' * A) \ A' * b; % \: inv
    corner = w';
end