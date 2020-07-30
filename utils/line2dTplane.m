% line_model: (a, b, c) st. a*col + b*row + c = 0
% plane_model: b is a 4x1 array of plane coefficients in the form
%              b(1)*X + b(2)*Y +b(3)*Z + b(4) = 0
%              The magnitude of b is 1.
function plane_model = line2dTplane(line_model, C)

% pt2d is in x-y coordinate (col-row)
pt2d1 = [0, -line_model(3)/line_model(2)];
pt2d2 = [C(1,3), -(line_model(1) * C(1,3) + line_model(3))/line_model(2)];

z = 100;
proj = [pt2d1(1)*z; pt2d1(2)*z; z];
pt3d1 = C\proj;

proj = [pt2d2(1)*z; pt2d2(2)*z; z];
pt3d2 = C\proj;

z = 200;
proj = [pt2d2(1)*z; pt2d2(2)*z; z];
pt3d3 = C\proj;

XYZ = [pt3d1 pt3d2 pt3d3];
plane_model = fitplane(XYZ);

end