% return the distance between two parallel planes
% input: 
% p1, p2 - plane model to the planes ax+by+cz+d=0
% points1, points2 - points on the planes
function dist = plane_dist(p1, p2, points1, points2)

n1 = p1(1:3);
n1 = n1./norm(n1);
n2 = p2(1:3);
n2 = n2./norm(n2);
p1 = mean(points1);
p2 = mean(points2);

num_points1 = size(points1, 1);
sum_dist1 = 0;
for i = 1:num_points1
    l = p2 - points1(i,:);
    sum_dist1 = sum_dist1 + abs(dot(l, n2));
end
dist1 = sum_dist1./num_points1;

num_points2 = size(points2, 1);
sum_dist2 = 0;
for i = 1:num_points2
    l = p1 - points2(i, :);
    sum_dist2 = sum_dist2 + abs(dot(l, n1));
end
dist2 = sum_dist2./num_points2;

dist = (dist1+dist2)./2;
end
