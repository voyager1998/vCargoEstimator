% Refer to the deduction below
function ptcloud = tof2pc(D, C)
s = size(D);
w = s(1);
h = s(2);
ptcloud = zeros(w*h,3);
n = 0;
for i = 1: w
    for j = 1: h
        if D(i,j) > 1000
            continue;
        end
        costheta = 1/sqrt( ((i-C(1,3))/C(1,1))^2 + ((j-C(2,3))/C(2,2))^2 + 1);
        z = double(D(i,j))*costheta;
        proj = [i*z; j*z; z];
        pos = C\proj;

        n = n+1;
        ptcloud(n,:) = pos';
    end
end
ptcloud = ptcloud(1:n, :);
end


% C*[x;y;z] = [px; py; z];
% px = fx*x+offx*z;
% py = fy*y+offy*z;
% [px/z; py/z] = [i;j];
% [px/z; py/z] = [fx*x/z + offx; fy*y/z+offy];
% [fx*x/z; fy*y/z] = [i-offx; j-offy];
% x = (i-offx)*z/fx;
% y = (j-offy)*z/fy;
% d = sqrt(x^2+y^2+z^2);
% cos(theta) = z/d = 1/sqrt( ((i-offx)/fx)^2 + ((j-offy)/fy)^2 + 1)
% z = d*cos(theta)