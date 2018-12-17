function [LandMarksComputed, AllPosesComputed] = SLAMusingGTSAM(DetAll, K, TagSize, qIMUToC, TIMUToC,...
                                                IMU, LeftImgs, TLeftImgs, Mode)
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
    for i = 1:length(frame_files)
        base_file_name = frame_files(i).name;
        full_file_name = fullfile(frames_folder, base_file_name);
        frame = imread(full_file_name);
        frames{i} = frame;
    end
    
    % Load mapping data
    mapping_data = load('DataMapping.mat');
    frames_detections = mapping_data.DetAll;
    frame_one_detections = frames_detections{1};
    
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
    tag_10_coords = [p1x, p1y; p2x, p2y; p3x, p3y; p4x, p4y];
    world_origin = [0, 0; TagSize, 0; TagSize, TagSize; 0, TagSize];
    
    % Initialize camera pose by calculating homography

end

function H = homography2d(varargin)
    
    [x1, x2] = checkargs(varargin(:));

    % Attempt to normalise each set of points so that the origin 
    % is at centroid and mean distance from origin is sqrt(2).
    [x1, T1] = normalise2dpts(x1);
    [x2, T2] = normalise2dpts(x2);
    
    % Note that it may have not been possible to normalise
    % the points if one was at infinity so the following does not
    % assume that scale parameter w = 1.
    
    Npts = length(x1);
    A = zeros(3*Npts,9);
    
    O = [0 0 0];
    for n = 1:Npts
	X = x1(:,n)';
	x = x2(1,n); y = x2(2,n); w = x2(3,n);
	A(3*n-2,:) = [  O  -w*X  y*X];
	A(3*n-1,:) = [ w*X   O  -x*X];
	A(3*n  ,:) = [-y*X  x*X   O ];
    end
    
    [U,D,V] = svd(A,0); % 'Economy' decomposition for speed
    
    % Extract homography
    H = reshape(V(:,9),3,3)';
    
    % Denormalise
    H = T2\H*T1;
