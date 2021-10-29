% Specify the folder where the files live.
myFolder = 'data/aditof/ir_images';
% myFolder = '/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/var345';
% Check to make sure that folder actually exists.  Warn user if it doesn't.
if ~isfolder(myFolder)
    errorMessage = sprintf('Error: The following folder does not exist:\n%s\nPlease specify a new folder.', myFolder);
    uiwait(warndlg(errorMessage));
    myFolder = uigetdir(); % Ask for a new one.
    if myFolder == 0
         % User clicked Cancel
         return;
    end
end
% Get a list of all files in the folder with the desired file name pattern.
filePattern = fullfile(myFolder, '*depth.png'); % Change to whatever pattern you need.
theFiles = dir(filePattern);
load('aditof_IR.mat');
for k = 1 : length(theFiles)
    baseFileName = theFiles(k).name;
    fullFileName = fullfile(theFiles(k).folder, baseFileName);
    fprintf(1, 'Now reading %s\n', fullFileName);
    % Now do whatever you want with this file name,
    % such as reading it in as an image array with imread()
    imageArray = imread(fullFileName);
    [rect_image,newOrigin] = undistortImage(imageArray,aditof_IR);
    imwrite(rect_image,fullFileName);
    imshow(rect_image);  % Display image.
    drawnow; % Force display to update immediately.
end