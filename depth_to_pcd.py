import numpy as np
import cv2
import torch 
import open3d as o3d

# depth_img = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/depth_data/0004_depth.png"
depth_img = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/depth_pico_3.png"
# depth_img = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/var345/test3_depth.png"
# pcd_depth = "/home/funderburger/work_ws/calibration_ws/planes_extr_ws/training_data/pcd_data/0004_pcd.pcd"

# max_depth = 7000
depth_np = cv2.imread(depth_img,-1)
depth_np = cv2.resize(depth_np,(640,360))
depth_np = np.array(depth_np,np.float32,copy=True)[:, :, None].transpose((2,0,1))
depth = torch.from_numpy(depth_np).long()#/max_depth
depth = depth.to('cuda')

eps = 1e-7

# depth[depth==0] = eps

cx = 334.081
cy = 169.808
fx = 460.585
fy = 460.268

# # # from matlab
# cx = 332.308128574715
# cy = 169.771477845517
# fx = 464.701646592003
# fy = 466.814346860831

# aditof camera
# cx = 315.611
# cy = 234.363
# fx = 372.7319
# fy = 374.4955
# cx = 313.595
# cy = 233.023
# fx = 377.22
# fy = 377.111

# ok for aditof
# cx = 330.4446
# cy = 228.6992
# fx = 377.22
# fy = 377.111

rows, cols = depth[0].shape
c, _ = torch.meshgrid(torch.arange(cols), torch.arange(cols))
c = torch.meshgrid(torch.arange(cols))
new_c = c[0].reshape([1,cols]).to('cuda')
r = torch.meshgrid(torch.arange(rows))
new_r = r[0].unsqueeze(-1).to('cuda')
valid = (depth[0] > 0) & (depth[0] < 65535)
nan_number = torch.tensor(np.nan).to('cuda')
zero_number = torch.tensor(0.).to('cuda')
z = torch.where(valid, depth[0]/1000.0, nan_number) ### / 1000.0
x = torch.where(valid, z * (new_c - cx) / fx, nan_number)
y = torch.where(valid, z * (new_r - cy) / fy, nan_number)

dimension = rows * cols
z_ok = z.reshape(dimension)
x_ok = x.reshape(dimension)
y_ok = y.reshape(dimension)

pcd_from_img = torch.stack((x_ok,y_ok,z_ok),dim=1)
pcd_from_img_np = pcd_from_img.cpu().detach().numpy()

# pcd_load = o3d.io.read_point_cloud(pcd_depth)
# xyz_load = np.asarray(pcd_load.points)

# open3d_img = o3d.geometry.Image(depth[0].cpu().detach().numpy())   
# intrinsics = o3d.cpu.pybind.camera.PinholeCameraIntrinsic(640,360,fx,fy,cx,cy) 
# pcd = o3d.geometry.PointCloud.create_from_depth_image(open3d_img,intrinsic=intrinsics)

pcd_file = open("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/pico_pcd3.txt","w")
# for row in pcd_from_img_np:
#     np.savetxt(pcd_file,row)
# pcd_file.close()
np.savetxt(pcd_file, pcd_from_img_np, delimiter=" ")

man_pcd = o3d.geometry.PointCloud()
man_pcd.points = o3d.utility.Vector3dVector(pcd_from_img_np)#*max_depth)
o3d.visualization.draw_geometries([man_pcd])
o3d.io.write_point_cloud("/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/pico/rgb_pcd/pcd_1.pcd",man_pcd,write_ascii=True)


print()