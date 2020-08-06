% Return the distance from the point p to the line
% line model(1:6) = [point on line, direction vector of line]
function dist = point_line_dist(P, line_model)
I = line_model(1:3);
u = line_model(4:6);
d = P - I;
proj = dot(d,u);
Q = I+proj*u;
dist = norm(P-Q);
end