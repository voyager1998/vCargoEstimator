
function projection = proj2plane(points, plane)
% given an plane equation ax+by+cz= -d, project points xyz onto the plane
% return the coordinates of the new projected points
% written by Neo Jing Ci, 11/7/18, revised by ZHU Yilun
projection = points;
a = plane(1,1);
b = plane(1,2);
c = plane(1,3);
d = plane(1,4);
A=[1 0 0 -a; 0 1 0 -b; 0 0 1 -c; a b c 0];

for i = 1:size(points,1)
    x = points(i,1);
    y = points(i,2);
    z = points(i,3);
    B=[x; y; z; -d];
    X=A\B;
    px=X(1);
    py=X(2);
    pz=X(3);  

    projection(i,:) = [px py pz];  

end



end
