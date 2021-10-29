function create_RGBD_PCD(scan_filename_colorizer, image_filename_colorizer, Kc, R, t)

tic
% load scan
scan3d=load(scan_filename_colorizer);
n_points=size(scan3d,1);
%scan=(handles.R2*scan'+repmat(handles.t2,1,n_points))';
% Convert to mm's
% scan3d=scan3d./1.0e2;

% transform in m
scan3d = scan3d(:,1:3)*1000;
xyz = scan3d(:,1:3);
% xyz = [ xyz(:,1) -xyz(:,3) xyz(:,2) ];

P = Kc * R * [eye(3) t];
% P = [R t'] * Kc;
abc = P * [xyz ones(size(xyz,1),1)]';
ab = zeros(3, length(abc));

for i = 1:length(ab)
   ab(:,i) = round(abc(:,i) / abc(3,i));
   if ( (ab(1,i) <= 0) || (ab(2,i) <= 0) )
       ab(1,i) = 100;
       ab(2,i) = 100;
   end
end
% scan=project_points2(scan',zeros(3,1),zeros(3,1),...
%     handles.camcalibparams.fc,handles.camcalibparams.cc,...
%     handles.camcalibparams.kc,handles.camcalibparams.alpha_c);
% Flip rows<->columns to get matlab image coordinates, and round off values
scan=fliplr(round(ab(1:2,:)'));

% Read image
% load('aditof_rect_640x480.mat'.mat');
% image=imread(image_filename_colorizer);
% [rect_image,newOrigin] = undistortImage(image,aditof_rect_640x480.CameraParameters2);
% imshow(rect_image);
rect_image=imread(image_filename_colorizer);

% Initialize empty matrix representing default point color=black
scanRGB=zeros(n_points,3);
imrows=size(rect_image,1);
imcols=size(rect_image,2);

% Find indices of all points that project within image
inliers=find(scan(:,1)>0 & scan(:,1)<imrows & scan(:,2)>0 ...
    & scan(:,2)<imcols);

%% For all points that project within the image, lookup the color
%% and store in scanRGB
% Convert [scan(inliers,1) scan(inliers,2)] to linear index based on size
% of image
inliers_lindex=sub2ind([imrows imcols],scan(inliers,1),scan(inliers,2));
% Convert image from imrows*imcols*3 to (imrows*imcols)*3
rect_image=reshape(rect_image,imrows*imcols,3);
scanRGB(inliers,:)=rect_image(inliers_lindex,:);
scanRGB = cast(scanRGB,'uint32');
% scanRGB( find(scan3d(:,1) > 5),: ) = 0;
clear scan image inliers inliers_lindex;

%% Write VRML file as output
fprintf(1,'Writing VRML file rgbScan.wrl\n');
fprintf(1,'This may take a minute, so please wait... ');
scanRGB_ok = cast(bitor(bitor(bitshift(scanRGB(:,1),16), bitshift(scanRGB(:,2),8)), scanRGB(:,3)),'uint32');
scanRGB_float = typecast(scanRGB_ok(:,1),'single');
% scanRGB_float = zeros(length(scanRGB_ok),1);
% for i=1:length(scanRGB_ok)
%     float_element = typecast([scanRGB_ok(i,1) 0],'double');
%     float_element = swapbytes(float_element);
%     scanRGB_float(i,1) = float_element;
% end
rgb_pcd_pico = [cast(scan3d,'single'), scanRGB_float];
% writematrix(rgb_pcd_pico,'rgbpcd_aditof_rect_640x480_Kc_corrected_3.pcd','Delimiter',' ','FileType','text');
writematrix(rgb_pcd_pico,'rgbpcd_pico_ros_stereo.pcd','Delimiter',' ','FileType','text');
% xyzrgb2vrml('rgbScan.wrl',scan3d,scanRGB);
% xyzrgb2pcd('rgbScan.wrl',scan3d,scanRGB);
fprintf(1,'Done!\n');
toc
return;