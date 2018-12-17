function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, Mode)
    % For Input and Output specifications refer to the project pdf

    import gtsam.*
    % Refer to Factor Graphs and GTSAM Introduction
    % https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
    % and the examples in the library in the GTSAM toolkit. See folder
    % gtsam_toolbox/gtsam_examples

    % Load frames containing April tags
    frames_folder = '~/Documents/cmsc426/final_project/MappingFrames/';
    file_pattern = fullfile(frames_folder, '*.jpg');
    frame_files = dir(file_pattern);
    frames = cell(length(frame_files));
    for i = 1:1
        base_file_name = frame_files(i).name;
        full_file_name = fullfile(frames_folder, base_file_name);
        frame = imread(full_file_name);
        frames{i} = frame;
    end
    
    % Load mapping data
    frame_one_detections = DetAll{1};
    
    % Detection data stores as [TagID, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y]
    first_col = frame_one_detections(:, 1);
    tag_10_data = frame_one_detections(first_col == 10, :);
    p1x = tag_10_data(2);
    p1y = tag_10_data(3);
    p2x = tag_10_data(4);
    p2y = tag_10_data(5);
    p3x = tag_10_data(6);
    p3y = tag_10_data(7);
    p4x = tag_10_data(8);
    p4y = tag_10_data(9);
    tag_10_coords = [p1x p1y; p2x p2y; p3x p3y; p4x p4y];
    world_origin = [0 0; TagSize 0; TagSize TagSize; 0 TagSize];
%     tag_10_coords = [p1x, p2x, p3x, p4x; p1y, p2y, p3y, p4y; 1, 1, 1, 1];
%     world_origin = [0, TagSize, TagSize, 0; 0, 0, TagSize, TagSize; 1, 1, 1, 1];
    
    % Initialize camera pose by calculating homography
    tform = estimateGeometricTransform(tag_10_coords, world_origin, 'projective')
    KH = tform.T;
    H = K \ KH % Equivalent to H = inv(K) * KH
    h_1 = H(:, 1);
    h_2 = H(:, 2);
    h_3 = H(:, 3);
    % SVD on H to find the Rotation (R) and Translation (T) values
    [U, S, V] = svd([h_1, h_2, cross(h_1, h_2)]);
    R = U * S * V;
    T = h_3 / norm(h_1);
    pose = [R, T];
%     [x, y] = transformPointsForward(H, p1x, p1y)
%     KH = homography2d(tag_10_coords, world_origin)
%     KH * [p4x; p4y; 1]
    H_check = [];
%     pose = [];
%     hold on;
%     for i = 1:size(frame_one_detections, 1)
%         tag_data = frame_one_detections(i, :);
%         p1x = tag_data(2);
%         p1y = tag_data(3);
%         p2x = tag_data(4);
%         p2y = tag_data(5);
%         p3x = tag_data(6);
%         p3y = tag_data(7);
%         p4x = tag_data(8);
%         p4y = tag_data(9);
%         tag_xs = [p1x p2x p3x p4x];
%         tag_ys = [p1y p2y p3y p4y];
%         [xs, ys] = transformPointsForward(tform, tag_xs', tag_ys');
%         %H_check = [H_check; xs(1), ys(1), xs(2), ys(2), xs(3), ys(3), xs(4), ys(4)];\
%         x = [xs(1) xs(2) xs(3) xs(4) xs(1)];
%         y = [ys(1) ys(2) ys(3) ys(4) ys(1)];
%         plot(x, y, 'b-', 'LineWidth', 2);
%     end
%     hold off;
    
end
