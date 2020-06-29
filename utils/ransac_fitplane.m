function [plane_model, inliers, inlier_indexs] ...
    = ransac_fitplane(pointcloud, noise_ths, iterations, subset_size, fitted_n)
    
    s = size(pointcloud,1);
    subset_size = min(subset_size, s);
    homo_points = [pointcloud ones(s,1)]';
    max_obj = 0;
    inlier_indexs  = [];
    for i=1:iterations
        samples = datasample(pointcloud, subset_size);
        p = fitplane(samples');
        n = p(1:3)/norm(p(1:3));
        res = abs((p'*homo_points)./norm(p(1:3)));
        inliers = res < noise_ths;
        num_inliers = sum(inliers);
        if isempty(fitted_n)
            lean = 1;
        else
            lean = abs(n'*fitted_n-0.5)+0.5;
        end
        obj = num_inliers;
        if obj > max_obj
            max_obj = obj;
            inlier_indexs = inliers;
        end
    end
    inliers = pointcloud(inlier_indexs, :);
    max_sample = 1000;
    if length(inliers) > max_sample
        samples = datasample(inliers, max_sample);
    else
        samples = inliers;
    end
    plane_model = fitplane(samples')';
    n = norm(plane_model(1:3));
    plane_model = plane_model/n;
end