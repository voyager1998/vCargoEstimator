function ptcloud = tof2pc_mat(D, C)
    h = size(D,1);
    w = size(D,2);
    loc = D > 3000;
    row_i = repmat(1:h,w,1)';
    col_i = repmat(1:w,h,1);
    costheta = 1./sqrt(((col_i-C(1,3))/C(1,1)).^2+((row_i-C(2,3))/C(2,2)).^2+1);
    z = double(D).*costheta;
    proj = [reshape(col_i.*z,1,w*h);reshape(row_i.*z,1,w*h);reshape(z,1,w*h)];
    ptcloud = (C\proj)';
    ptcloud(loc,:) = [];
end