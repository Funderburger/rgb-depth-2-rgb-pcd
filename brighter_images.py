from PIL import Image, ImageEnhance
import os
import matplotlib.pyplot as plt
import cv2
# import imgaug as imaug

# flip an image
# image = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0018/testing_images/depth_test_9.png"
# depth_img = cv2.imread(image,-1)
# plt.imshow(depth_img)
# plt.colorbar()
# plt.close()

# def vertical_flip(img,img_name):
#     flipped_img = imaug.augmenters.flipud(img)
#     cv2.imwrite(img_name[:-4]+"_ver_flip.png",flipped_img)

# vertical_flip(depth_img, image)


######################################################
#  brighter images pilLOW

# image_dir = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0018_v2/capturi/rgb/"

# for image_path in os.listdir(image_dir):

#     im = Image.open(image_dir+image_path)
#     enhancer = ImageEnhance.Brightness(im)

#     factor = 5
#     im_output = enhancer.enhance(factor)

#     im_output.save(image_dir+"brighter_"+image_path)
#     break

######################################################
#  brighter images OpenCV

image_dir = "/home/funderburger/work_ws/calibration_ws/camera_cross_calib_ws/new_dataset/cam_P0018_v3/capturi/test/"

for image_path in os.listdir(image_dir):

    image = cv2.imread(image_dir+image_path,-1)

    alpha = 0.01 # Simple contrast control
    beta = 1    # Simple brightness control
    # Initialize values

    # Do the operation new_image(i,j) = alpha*image(i,j) + beta
    # Instead of these 'for' loops we could have used simply:
    new_image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    # but we wanted to show you how to access the pixels :)
    # for y in range(image.shape[0]):
    #     for x in range(image.shape[1]):
    #         for c in range(image.shape[2]):
    #             new_image[y,x,c] = np.clip(alpha*image[y,x,c] + beta, 0, 255)
    cv2.imwrite(image_dir+"brighter_"+image_path, new_image)

print("Done!")