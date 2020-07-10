function upper_edge = detect_ROIedge(upper_pts,D_denoise)
%% parameter to adjust
enlarge_range = 15; % manually make the range larger
edge_thres = 0.1;

%% find region of interest
upper_pts = round(upper_pts);
upper_2D = zeros(size(D_denoise)); % take the 3D points of upper plane to 2D
for i = 1:size(upper_pts, 1)
    upper_2D(upper_pts(i,2), upper_pts(i,1)) = 1;
end
upper_2D = logical(upper_2D); % change from double 0,1 to logical 0,1
upper_2D_post = imfill(upper_2D, 'holes'); % fill in the holes
upper_2D_post = bwareaopen(upper_2D_post, 10000); % reject small objects

% find the rectangle that contain the upper plane
stats = regionprops(upper_2D_post);

% notice BoundingBox =  [x y width height], but image is [col row], and col is reverse from y
col_min = round(stats.BoundingBox(2)) - enlarge_range;
col_max = round(stats.BoundingBox(2) + stats.BoundingBox(4)) + enlarge_range;
row_min = round(stats.BoundingBox(1)) - enlarge_range;
row_max = round(stats.BoundingBox(1) + stats.BoundingBox(3)) + enlarge_range;
if col_min<1
    col_min=1;
end
if col_max>480
    col_max=480;
end
if row_min<1
    row_min=1;
end
if row_max>640
    row_max=640;
end
D_smallPlane = D_denoise(col_min:col_max, row_min:row_max);

D_smallEdge = edge(D_smallPlane, 'Canny', edge_thres); % edge detection on the small portion of image

% put back to 640*480 image size
D_edge = zeros(size(D_denoise));
D_edge(col_min:col_max, row_min:row_max) = D_smallEdge;
upper_edge = bwareafilt(logical(D_edge),1); % always take out the biggest, somewhat brute force

end