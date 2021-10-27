clear all;
load('picoCalibration.mat');
load('picoStereov2.mat');
load('aditof_Params.mat');
load('aditof_RGB.mat');
load('aditof_rect_640x480.mat');
format long
% Kc = picoStereov2.CameraParameters2.IntrinsicMatrix';
Kc = cameraPicoParams.CameraParameters2.IntrinsicMatrix';
% Kc = [523.463,0,318.238;0,523.132,191.743;0,0,1];


% rectified pairs at 640x480
% Kc = aditof_rect_640x480.CameraParameters2.IntrinsicMatrix';
% cx-11
% Kc = [3.085029193506431e+02,0,3.170166919331039e+02;0,4.124376270235626e+02,2.417330826416787e+02;0,0,1];

% pico_ros
% Kc = [524.2946151738875, 0.0, 314.52019927498145; 0.0, 524.8590836334387, 189.5955672789045; 0.0, 0.0, 1.0];

% Kc = [3.099654931647211e+02,0,0;0,4.151808530355780e+02,0;3.172214616744955e+02,2.423328133377825e+02,1]';
% rectified pairs at 640x360
% Kc = aditofParams.CameraParameters2.IntrinsicMatrix';

% Kc = aditof_RGB.IntrinsicMatrix'/3;
% Kc = cameraPicoParams.EssentialMatrix;
% Kc = [1046.9256587438058, 0.0, 636.475752614184; 0.0, 1046.2645954490595, 383.48569282367384; 0.0, 0.0, 1.0]/2;
% R = picoStereov2.RotationOfCamera2;
% R = cameraPicoParams.RotationOfCamera2';

% rectified pairs at 640x480
% R = aditof_rect_640x480.RotationOfCamera2';

% pico
R = [0.9996941314743334, 0.000594457761260526, 0.02472428190561326; -0.0009156735636596173, 0.9999153029107688, 0.012982624803568885; -0.024714470208826507, -0.013001293198582372, 0.9996100046205327]';
% rectified pairs at 640x360
% R = aditofParams.RotationOfCamera2';

% R = [0.999835, -0.00687601, -0.0168025; 0.00681042, 0.999969, -0.00395785; 0.0168292, 0.00384276, 0.999851];
% t = picoStereov2.TranslationOfCamera2;
t = cameraPicoParams.TranslationOfCamera2;

% rectified pairs at 640x480
% t = aditof_rect_640x480.TranslationOfCamera2;

% pico
% t = [-0.04812637132150153, -0.0014468796626424256, 0.0018017886081715772]*1000;
% rectified pairs at 640x360
% t = aditofParams.TranslationOfCamera2;

% t = [47.1934, 0.0297999, -1.67577];
create_RGBD_PCD('pico_pcd3.txt','rgb_pico_3.png',Kc,R,t')
% create_RGBD_PCD('pico_pcd_1.txt','/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/var345/res-test3_rgb.png',Kc,R,t')