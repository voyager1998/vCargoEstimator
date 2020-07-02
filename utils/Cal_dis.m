function [l, w, h] = Cal_dis(plane_points, plane_models)
    %% Calculate height
    
    % here, use cell (rather than array or matrix) for potential future indexing
    n_unnorm{1}= plane_models(1,1:3); % unnormalized normal vector
    n{1} = n_unnorm{1} / norm(n_unnorm{1}); % normalized normal vector (both are useful)
    
    n_unnorm{2} = plane_models(2,1:3);
    n{2} = n_unnorm{2} / norm(n_unnorm{2});
    
    n_unnorm{3} = plane_models(3,1:3);
    n{3} = n_unnorm{3} / norm(n_unnorm{3});
    
    n_unnorm{4} = plane_models(4,1:3);
    n{4} = n_unnorm{4} / norm(n_unnorm{4});  
    
    plane_index = 1:4; % initialize the index for 4 planes          
    ground_index = 1; % Key assumption: ground is the first fitted plane
    plane_index(plane_index == ground_index) = []; % be careful with the syntax in the backet
    
    normal_dp = abs([n{1}*n{2}', n{1}*n{3}', n{1}*n{4}']); % calculating the dot product of normal vectors    

    upper_index = find(normal_dp == max(normal_dp)) + 1; % get index for upper plane (prependicular to ground)
    plane_index(plane_index == upper_index) = [];
    
    left_index = plane_index(1); right_index = plane_index(2); % sequence of left and right don't matter
    
    ground_homo = [plane_points{ground_index} ones(size(plane_points{ground_index},1), 1)];
    upper_homo = [plane_points{upper_index} ones(size(plane_points{upper_index},1), 1)];
    
    h1 = abs(upper_homo * plane_models(ground_index,:)') / norm(n_unnorm{ground_index});
    h2 = abs(ground_homo * plane_models(upper_index,:)') / norm(n_unnorm{upper_index});
    
    h = (sum(h1)+sum(h2))/(length(h1)+length(h2));   
   
    %% Calculate length and width
    
     origin = plane_models(2:4,1:3) \-plane_models(2:4,4);
     dir_l = cross(n{ground_index}, n{left_index}); % dirction of left and right   
     dir_r = cross(n{ground_index}, n{right_index});
     upper_shift = plane_points{upper_index} - repmat(origin', size(plane_points{upper_index},1), 1);
     data_l = abs(upper_shift * dir_l'); % project the data on upper plane to the direction of left and right
     data_r = abs(upper_shift * dir_r');
     max_l = max(data_l);
     max_r = max(data_r);
     
     l = max(max_l, max_r); % length is the bigger value
     w = min(max_l, max_r);  
    
end
    
