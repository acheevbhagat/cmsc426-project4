function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, Mode)
    % For Input and Output specifications refer to the project pdf

    import gtsam.*
    % Refer to Factor Graphs and GTSAM Introduction
    % https://research.cc.gatech.edu/borg/sites/edu.borg/files/downloads/gtsam.pdf
    % and the examples in the library in the GTSAM toolkit. See folder
    % gtsam_toolbox/gtsam_examples

    LandMarksComputed = [];
    AllPosesComputed = [];
    
    % Load frames containing April tags
    frames_folder = '../MappingFrames/MappingFrames/';
    file_pattern = fullfile(frames_folder, '*.jpg');
    frame_files = dir(file_pattern);
    numfiles = length(frame_files);
    frames = cell(numfiles);
    for f = 1:numfiles
        frame = imread(fullfile(frames_folder, frame_files(f).name));
        frames{f} = frame;
    end
    
    
    homographies = cell(numfiles, 1);
    
    % Load mapping data
    frame_one_detections = DetAll{1};
    
    % Detection data stored as [TagID, p1x, p1y, p2x, p2y, p3x, p3y, p4x, p4y]
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
    origin_coords = [0 0; TagSize 0; TagSize TagSize; 0 TagSize];
    %tag_10_coords = [[p1x p1y 1]' [p2x p2y 1]' [p3x p3y 1]' [p4x p4y 1]'];
    %origin_coords = [[0 0 1]' [TagSize 0 1]' [TagSize TagSize 1]' [0 TagSize 1]'];
    
    % Initialize camera pose by calculating homography
    tform = estimateGeometricTransform(tag_10_coords, origin_coords, 'projective');
    homographies{1} = tform.T;
    
%     KH = tform.T;
%     %KH = homography2d(tag_10_coords, inv(K) * origin_coords);
%     %KH * tag_10_coords(:, 1)
%     H = K \ KH % Equivalent to H = inv(K) * KH
%     h_1 = H(:, 1);
%     h_2 = H(:, 2);
%     h_3 = H(:, 3);
%     % SVD on H to find the Rotation (R) and Translation (T) values for pose
%     [U, S, V] = svd([h_1 h_2 cross(h_1, h_2)]);
%     R = U * [1 0 0; 0 1 0; 0 0 det(U * V')] * V';
%     T = h_3 / norm(h_1);
%     pose = [R T]
    
    %world_frame_coords = [10, 0, 0, TagSize, 0, TagSize, TagSize, 0, TagSize];
    %imshow(frames{1});
    hold on;
    prev_world_coords = [];
    img_pts = [];
    world_pts = [];
    for f = 1:size(frame_one_detections, 1)
        tag_data = frame_one_detections(f, :);
        tag_id = tag_data(1);
        p1x = tag_data(2);
        p1y = tag_data(3);
        p2x = tag_data(4);
        p2y = tag_data(5);
        p3x = tag_data(6);
        p3y = tag_data(7);
        p4x = tag_data(8);
        p4y = tag_data(9);
        tag_xs = [p1x p2x p3x p4x];
        tag_ys = [p1y p2y p3y p4y];
        img_pts = [img_pts; p1x p1y; p2x p2y; p3x p3y; p4x p4y];
        % Code to check homography accuracy
        [xs, ys] = transformPointsForward(tform, tag_xs', tag_ys');
        world_pts = [world_pts; xs(1) ys(1) 0; xs(2) ys(2) 0; xs(3) ys(3) 0; xs(4) ys(4) 0];
        x = [xs(1) xs(2) xs(3) xs(4) xs(1)];
        y = [ys(1) ys(2) ys(3) ys(4) ys(1)];
        prev_world_coords = [prev_world_coords; tag_id xs(1) ys(1) xs(2) ys(2) xs(3) ys(3) xs(4) ys(4)];
        %plot(x, y, 'b-', 'LineWidth', 2);
    end
    
    params = cameraParameters('IntrinsicMatrix', K');
    [R, T] = estimateWorldCameraPose(img_pts, ...
    world_pts, params, 'Confidence', 70);
    pose = [R T'];
    
    quat = v_rotro2qr(R);
    AllPosesComputed = [AllPosesComputed; T(1) T(2) T(3) quat(1) quat(2) quat(3) quat(4)];
    hold off;
    LandMarksComputed = [LandMarksComputed; prev_world_coords];
    
    % Run through the images and update on each image
    for f = 2:numfiles
        %disp(f)
        % Find common landmarks between the frames
        currLandmarks = DetAll{f};
        prevLandmarks = prev_world_coords;
        commonTags = [];
        for j = 1:size(currLandmarks, 1)
            currLandmarkID = currLandmarks(j, 1);
            for k = 1:size(prevLandmarks, 1)
                prevLandmarkID = prevLandmarks(k, 1);
                if currLandmarkID == prevLandmarkID
                    commonTags = [commonTags currLandmarkID];
                    break;
                end
            end
        end
        commonTags = sort(commonTags);
        currLandmarks = sortrows(currLandmarks, 1);
        prevLandmarks = sortrows(prevLandmarks, 1);
        % Calculate homography between all sets of landmarks
        % Pull out landmark information for each landmark
        currInfoX = [];
        currInfoY = [];
        prevInfoX = [];
        prevInfoY = [];
        currFirstCol = currLandmarks(:, 1);
        prevFirstCol = prevLandmarks(:, 1);
        for j = 1:length(commonTags)
            currInfoVec = currLandmarks(currFirstCol == commonTags(j), :)';
            prevInfoVec = prevLandmarks(prevFirstCol == commonTags(j), :)';
            % First column X values
            currInfoX = [currInfoX currInfoVec(2) currInfoVec(4) ...
                currInfoVec(6) currInfoVec(8)];
            prevInfoX = [prevInfoX prevInfoVec(2) prevInfoVec(4) ...
                prevInfoVec(6) prevInfoVec(8)];
            % Second column Y values
            currInfoY = [currInfoY currInfoVec(3) currInfoVec(5) ...
                currInfoVec(7) currInfoVec(9)];
            prevInfoY = [prevInfoY prevInfoVec(3) prevInfoVec(5) ...
                prevInfoVec(7) prevInfoVec(9)];
        end
        % Shape the data to a 3 x N for homography2d
        % Make third row of ones
        currInfo = [   ...
            currInfoX; ...
            currInfoY; ...
            ];
        prevInfo = [   ...
            prevInfoX; ...
            prevInfoY; ...
            ];
        
        %H = homography2d(currInfo, prevInfo);
        tform = estimateGeometricTransform(currInfo', prevInfo', 'projective');
        homographies{f} = tform.T;
        
        prev_world_coords = [];
        img_pts = [];
        world_pts = [];
        for i = 1:size(currLandmarks, 1)
            tag_data = currLandmarks(i, :);
            tag_id = tag_data(1);
            p1x = tag_data(2);
            p1y = tag_data(3);
            p2x = tag_data(4);
            p2y = tag_data(5);
            p3x = tag_data(6);
            p3y = tag_data(7);
            p4x = tag_data(8);
            p4y = tag_data(9);
            img_pts = [img_pts; p1x p1y; p2x p2y; p3x p3y; p4x p4y];
            tag_xs = [p1x p2x p3x p4x];
            tag_ys = [p1y p2y p3y p4y];
            [xs, ys] = transformPointsForward(tform, tag_xs', tag_ys');
            world_pts = [world_pts; xs(1) ys(1) 0; xs(2) ys(2) 0; xs(3) ys(3) 0; xs(4) ys(4) 0];
            x = [xs(1) xs(2) xs(3) xs(4) xs(1)];
            y = [ys(1) ys(2) ys(3) ys(4) ys(1)];
            prev_world_coords = [prev_world_coords; tag_id xs(1) ys(1) xs(2) ys(2) xs(3) ys(3) xs(4) ys(4)];
            %plot(x, y, 'b-', 'LineWidth', 2);
        end
        
        params = cameraParameters('IntrinsicMatrix', K');
        [R, T] = estimateWorldCameraPose(img_pts, ...
            world_pts, params, 'Confidence', 70);
        pose = [R T'];

        quat = v_rotro2qr(R);
        AllPosesComputed = [AllPosesComputed; T(1) T(2) T(3) quat(1) quat(2) quat(3) quat(4)];
        
        for p = 1:size(prev_world_coords, 1)
            if ~ismember(prev_world_coords(p, 1), LandMarksComputed(:, 1))
                LandMarksComputed = [LandMarksComputed; prev_world_coords(p, :)];
            end
        end
    end
    
    hold on;
    % Plot landmarks
%     for p = 1:size(LandMarksComputed, 1)
%         tag_data = LandMarksComputed(p, :);
%         p1x = tag_data(2);
%         p1y = tag_data(3);
%         p2x = tag_data(4);
%         p2y = tag_data(5);
%         p3x = tag_data(6);
%         p3y = tag_data(7);
%         p4x = tag_data(8);
%         p4y = tag_data(9);
%         tag_xs = [p1x p2x p3x p4x p1x];
%         tag_ys = [p1y p2y p3y p4y p1y];
%         plot(tag_xs, tag_ys, 'b-', 'LineWidth', 2);
%     end
%     
%     % Plot pose
%     for p = 1:size(AllPosesComputed, 1)
%         pose_data = AllPosesComputed(p, :);
%         x = pose_data(1);
%         y = pose_data(2);
%         plot(x, y, 'ro', 'LineWidth', 2);
%     end

    for p = 1:size(LandMarksComputed, 1)
        tag_data = LandMarksComputed(p, :);
        p1x = tag_data(2);
        p2x = tag_data(4);
        p3x = tag_data(6);
        p4x = tag_data(8);
        tag_xs = [p1x p2x p3x p4x p1x];
        tag_zs = [0 0 0 0 0];
        plot(tag_xs, tag_zs, 'b-', 'LineWidth', 2);
    end
    
    % Plot pose
    for p = 1:size(AllPosesComputed, 1)
        pose_data = AllPosesComputed(p, :);
        x = pose_data(1);
        z = pose_data(3);
       
        plot(x, z, 'ro', 'LineWidth', 2);
    end
    hold off;
    
    
    % Factor Graph Section
    x = cell(length(DetAll), 1);
    for f = 1:length(DetAll)
        x{f} = symbol('x', f);
    end
    
    % Variables to store landmark points
    all_frames_landmarks = cell(length(LandMarksComputed), 1);
    for f = length(DetAll)
        cur_frame_landmarks = sortrows(DetAll{f}, 1);
        p = cell(length(cur_frame_landmarks),1);
        point_count = 0;
        for f = 1:length(cur_frame_landmarks(:, 1))
            for j = 1:length(cur_frame_landmarks(f, 2:length()))
                point_count = point_count + 1;
                p{point_count} = symbol('lp', cur_frame_landmarks(point_count, 1));
            end
        end
        all_frames_landmarks{f} = p;
    end

    graph = NonlinearFactorGraph;

    % Add prior
    priorMean = Pose2(0.0, 0.0, 0.0); % prior at origin
    priorNoise = noiseModel.Diagonal.Sigmas([0.3; 0.3; 0.1]);
    graph.add(PriorFactorPose2(x{1}, priorMean, priorNoise));
    
    % Add odometry between steps
    odometryNoise = noiseModel.Diagonal.Sigmas([0.2 0.2 0.1]);
    for f = 1:length(x) - 1
        graph.add(BetweenFactorPose2(x{f}, x{f+1}, eye(3), odometryNoise));
    end
    
    % Add projection factor between pose and all landmark points
    for f = 1:length(all_frames_landmarks)
        for p = 1:length(all_frames_landmarks{f})
            graph.add(Point2) 
        end
    end
end
