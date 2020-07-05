% A pixel (i, j) in image, i -> y, j -> x
function ptcloud = pixel2pc(D, C)

corner_num = size(D,1);
ptcloud = zeros(corner_num,3);

for n = 1:corner_num % number of corners
    i = D(n,2);
    j = D(n,1);
    costheta = 1/sqrt( ((j-C(1,3))/C(1,1))^2 + ((i-C(2,3))/C(2,2))^2 + 1);
    z = double(D(n,3))*costheta;
    proj = [j*z; i*z; z];
    pos = C\proj;
    ptcloud(n,:) = pos';
end

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
% cos(theta) = z/d = 1/sqrt( ((j-offx)/fx)^2 + ((i-offy)/fy)^2 + 1)
% z = d*cos(theta)