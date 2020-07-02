% return the distance between two parallel lines
function dist = line_dist(u1,I1,u2,I2,points1,points2)
u1=u1./norm(u1);
u2=n2./norm(u2);
num_points1 = size(points1, 1);
sum_dist1 = 0;
for i = 1:num_points1
    sum_dist1 = sum_dist1 + cross((points1(i,:)-I2),u2);
end
dist1 = sum_dist1./num_points1;

num_points2 = size(points2, 1);
sum_dist2 = 0;
for i = 1:num_points2
    sum_dist2 = sum_dist2 + cross((points2(i,:)-I1),u1);
end
dist2 = sum_dist2./num_points2;

dist = (dist1+dist2)./2;
end
