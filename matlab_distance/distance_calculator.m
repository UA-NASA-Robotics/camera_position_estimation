numImages = 12;
files = cell(1, numImages);
D = 'C:\Users\david\OneDrive - The University of Akron\2020_Spring\Robotics\camera_position_estimation\matlab_distance';

for i = 1:numImages
    files{i} = fullfile(D, sprintf('image%d.jpg', i));
end

% Display one of the calibration images
%magnification = 25;
%I = imread(files{1});
%figure; imshow(I, 'InitialMagnification', magnification);
%title('One of the Calibration Images');

% Detect the checkerboard corners in the images.
[imagePoints, boardSize] = detectCheckerboardPoints(files);

% Generate the world coordinates of the checkerboard corners in the
% pattern-centric coordinate system, with the upper-left corner at (0,0).
squareSize = 29; % in millimeters
worldPoints = generateCheckerboardPoints(boardSize, squareSize);

% Calibrate the camera.
imageSize = [size(I, 1), size(I, 2)];
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
                                     'ImageSize', imageSize);

% Evaluate calibration accuracy.
%figure; showReprojectionErrors(cameraParams);
%title('Reprojection Errors');

imOrig = imread(fullfile(D, sprintf('test.jpg')));
%figure; imshow(imOrig, 'InitialMagnification', magnification);
%title('Input Image');

% Since the lens introduced little distortion, use 'full' output view to illustrate that
% the image was undistored. If we used the default 'same' option, it would be difficult
% to notice any difference when compared to the original image. Notice the small black borders.
[im, newOrigin] = undistortImage(imOrig, cameraParams, 'OutputView', 'full');
%figure; imshow(im, 'InitialMagnification', magnification);
%title('Undistorted Image');

% Detect the checkerboard.
[imagePoints, boardSize] = detectCheckerboardPoints(im);

% Adjust the imagePoints so that they are expressed in the coordinate system
% used in the original image, before it was undistorted.  This adjustment
% makes it compatible with the cameraParameters object computed for the original image.
imagePoints = imagePoints + newOrigin % adds newOrigin to every row of imagePoints

% Compute rotation and translation of the camera.
[R, t] = extrinsics(imagePoints, worldPoints, cameraParams);

image_location = [imagePoints(1,1) imagePoints(1,2)]

% Convert to world coordinates.
center1_world  = pointsToWorld(cameraParams, R, t, image_location);

% Remember to add the 0 z-coordinate.
center1_world = [center1_world 0];

% Compute the distance to the camera.
[~, cameraLocation] = extrinsicsToCameraPose(R, t);
distanceToCamera = norm(center1_world - cameraLocation);
fprintf('Distance from the camera to the first square is = %0.2f mm\n', ...
    distanceToCamera);

