% A pixel (i, j) in image, i -> y, j -> x
% pts: n x 2
function ptcloud = tofpts2pc(D, C, pts)
s = size(pts);
n = s(1);
ptcloud = zeros(n,3);
% TODO: instead of for loop, use matrix
for k = 1: n
    i = pts(k, 1);
    j = pts(k, 2);
    costheta = 1/sqrt( ((j-C(1,3))/C(1,1))^2 + ((i-C(2,3))/C(2,2))^2 + 1);
    z = double(D(i,j))*costheta;
    proj = [j*z; i*z; z];
    pos = C\proj;

    ptcloud(k,:) = pos';
end

end
