% irCameraParams = stereoParams.CameraParameters2
% rgbCameraParams = stereoParams.CameraParameters1
function [ptcloud, color] = tofRGB2pcColor(D, rgb, stereoParams)
    ptcloud = tof2pc(D, stereoParams.CameraParameters2.IntrinsicMatrix');
    color = zeros(size(ptcloud));
    color(:, 1) = 255;
%     pc_rgb = inv(stereoParams.RotationOfCamera2)*ptcloud'-stereoParams.TranslationOfCamera2';
    pc_rgb = stereoParams.RotationOfCamera2*ptcloud' ...
        + stereoParams.TranslationOfCamera2'; %+ [12; -19; 0];
    pc_rgb = pc_rgb';
    projected = stereoParams.CameraParameters1.IntrinsicMatrix' * pc_rgb';
    projected = projected';
    projected(:,1) = projected(:,1) ./ projected(:,3);
    projected(:,2) = projected(:,2) ./ projected(:,3);
    projected = projected(:, 1:2);
%     [projected, valid] = pc2pixel(pc_rgb, stereoParams.CameraParameters1.IntrinsicMatrix,...
%         [], [], stereoParams.CameraParameters1.ImageSize, false);
%     scatter(projected(:,1),projected(:,2),5,'filled');
%     title("reproject 2d prom rgb camera's perspective");
    for i = 1:size(projected, 1)
        if projected(i, 1) > 1 && projected(i, 1) <= size(rgb, 2) ...
                && projected(i, 2) > 1 && projected(i, 2) <= size(rgb, 1)
            color(i, :) = rgb(int64(projected(i, 2)), int64(projected(i, 1)), :);
        end
    end

end
