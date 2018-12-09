#!/usr/bin/env python
import numpy as np
import cv2
import matplotlib.pyplot as plt
import freenect
from sklearn.mixture import GaussianMixture
from scipy.ndimage.measurements import center_of_mass
from scipy.integrate import quad

def aquire_depth_map():
    # mdev = freenect.open_device(freenect.init(), 0)
    # freenect.set_depth_mode(mdev, freenect.RESOLUTION_HIGH, freenect.DEPTH_REGISTERED)
    # freenect.runloop(dev=mdev)
    # def display_depth(dev, data, timestamp):
    #     d = np.array(data.copy())
    #     # with open('depth_'+str(timestamp)+'.npy', 'w+') as f:
    #     #   np.save(f, d, allow_pickle=True)
    #     d[d>2000] = np.mean(d[d<2000].flatten())
    #     plt.imshow(d)
    #     plt.show()

    # # if cv2.WaitKey(10) == 27:
    # #   keep_running = False
    # # d = freenect.sync_get_depth()[0]
    # freenect.runloop(dev=mdev, depth=display_depth)
    # print(freenect.sync_get_depth())
    depth_map = np.array(freenect.sync_get_depth()[0].copy())
    return depth_map

def process_depth_map(raw_depth, plot):
    # plot histogram of values less than 2000
    # (values above 2000 are usually error values)
    # Lower mode corresponds to values in the box
    temp = raw_depth.flatten()
    raw_mean = np.mean(temp)
    # error correct
    ec_depth = temp[temp < raw_mean]
    var = np.var(ec_depth)
    mean = np.mean(ec_depth)

    # Set the errored values to a a bit above the average of the rest of the values
    d = raw_depth.copy()
    d[d > raw_mean] = mean + .01 * var
    if plot:
        plt.imshow(d)
        plt.show()


    # Apply Low Pass filter to remove noise
    dst = cv2.GaussianBlur(d,(15,15), 150)

    # Gaussian Mixture Model for predictions
    N = 2
    gmm = GaussianMixture(N, max_iter=25)
    gmm.fit(np.expand_dims(d.flatten(), axis=1))
    weights, means, variances = (gmm.weights_, gmm.means_, gmm.covariances_)
    means = means[:, 0]

    predictions = gmm.predict(dst.reshape(dst.shape[0] * dst.shape[1], 1))
    predictions = predictions.reshape(dst.shape[0], dst.shape[1])

    # Make sure the box is set to 1
    if (means[0] < means[1]):
        predictions = 1 - predictions

    if plot:
        plt.imshow(predictions)
        plt.show()

    non_zero = np.count_nonzero(predictions.flatten()) / float(dst.shape[0] * dst.shape[1])
    if non_zero > .2 or non_zero <.0045:
        raise ValueError("Box predicted to be " + str(non_zero)+"\% of image. Too large or non-existent")


    # Let's see what it thinks the box is

    return predictions

def get_angle(depth_map, center, plot=False):
# def get_angle(depth_map, center, rmin, rmax, cmin, cmax, plot=False):
    # # Get the locations the corners, and the angles from where they would be
    # # if the box was not rotated (wrt the sensor)
    # a1 = np.array([rmin, np.max(np.where(depth_map[rmin, :] != 0))]) - center
    # a1 = a1/np.linalg.norm(a1)
    # theta1 = np.arccos(np.dot(a1.T, np.array([-1, 1])/(2**(.5))))
    # # if theta1 > np.pi/4:
    # #     theta1 = np.pi/4 - theta1

    # a2 = np.array([rmax, np.min(np.where(depth_map[rmax, :] != 0))]) - center
    # a2 = a2/np.linalg.norm(a2)
    # theta2 = np.arccos(np.dot(a2.T, np.array([1, -1])/(2**(.5))))
    # # if theta2 / np.pi / 4:
    # #     theta2 = np.pi / 4 - theta2

    # a3 = np.array([np.min(np.where(depth_map[:, cmin] != 0)), cmin]) - center
    # a3 = a3/np.linalg.norm(a3)
    # theta3 = np.arccos(np.dot(a3.T, np.array([-1, -1])/(2**(.5))))
    # # if theta3 / np.pi / 4:
    # #     theta3 = np.pi / 4 - theta3

    # a4 = np.array([ np.max(np.where(depth_map[ :, cmax] != 0)), cmax]) - center
    # a4 = a4/np.linalg.norm(a4)
    # theta4 = np.arccos(np.dot(a4.T, np.array([1, 1])/(2**(.5))))
    # # if theta4 / np.pi / 4:
    # #     theta4 = np.pi / 4 - theta4

    # theta = np.mean([theta1, theta2, theta3, theta4])

    # Retrieves the integrand for our objective function

    def get_function_to_integrate(img, cmx, cmy, theta):
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        
        def integrand(x):
            # swap sin and cos cuz our x and y are swapped in visualizations
            x_val = cmx+int(x * sin_theta)
            y_val = cmy+int(x * cos_theta)
            if x_val < img.shape[0] and x_val >= 0 and y_val < img.shape[1] and y_val >= 0:
                return img[x_val, y_val]
            else:
                return 0
        return integrand

    # Decide the max number of samples and the max and minimum allowed angles (in radians)
    n_samples = 500
    max_angle = np.pi/2
    min_angle = -1* np.pi/2
    sample_rate = (max_angle - min_angle)/float(n_samples)

    def grid_search_angle(img, cmx, cmy, plot=False):
        lower, upper = (-1 * cmx, img.shape[0] - cmx)
        intersections = []
        thetas = np.arange(min_angle , max_angle + sample_rate, sample_rate)
        
        # iterate through thetas and compute the integral
        for theta in thetas:
            integrand = get_function_to_integrate(depth_map, center[0], center[1], theta)
            intersections.append((quad(integrand, lower, upper)[0], theta))
        if plot:    
            plt.plot(thetas * 180 /np.pi, np.array(intersections)[:,0])
            plt.show()
        
        # pick the minimizer
        return min(intersections, key = lambda x: x[0])


    result = grid_search_angle(depth_map, center[0], center[1], plot)
    print(result)
    print(result[1]*180 /np.pi)
    theta = result[1]
    if plot:
        temp = depth_map.copy()
        for i in np.arange(-100, 100):
            x_val = int(center[0] + i * np.sin(theta))
            y_val = int(center[1] + i * np.cos(theta))
            if x_val < depth_map.shape[0] and x_val >= 0 and y_val < depth_map.shape[1] and y_val >= 0:
                temp[x_val-4:x_val+4, y_val-4:y_val+4] = 2
        plt.imshow(temp)
        plt.show()
    return theta

def get_center(depth_map, plot):
    # Get the bounding box based off the non-zero values
    # def bbox2(img):
    #     rows = np.any(img, axis=1)
    #     cols = np.any(img, axis=0)
    #     occupied_rows = np.where(rows)[0]
    #     occupied_cols = np.where(cols)[0]
    #     if len(occupied_rows) == 0 or len(occupied_cols) == 0:
    #         # If either are empty, we need to return an error!
    #         raise ValueError("No box found")
    #     rmin, rmax = (occupied_rows[0], occupied_rows[-1])
    #     cmin, cmax = (occupied_cols[0], occupied_cols[-1])

    #     return rmin, rmax, cmin, cmax

    # rmin, rmax, cmin, cmax = bbox2(depth_map)
    # # print(rmin, rmax, cmin, cmax)
    # centery = int((cmin + cmax)/2)
    # centerx = int((rmin + rmax)/2)
    cm = np.array(center_of_mass(depth_map))

    temp = depth_map.copy()
    centerx, centery = int(cm[0]), int(cm[1])
    # temp[cmx-4:cmx+4, cmy-4:cmy+4] = 150

    # do [-4,+4] to make lines visible
    # temp[rmin-4:rmin+4, cmin:cmax] = 50
    # temp[rmax-4:rmax+4, cmin:cmax] = 50
    # temp[rmin:rmax, cmin-4:cmin+4] = 50
    # temp[rmin:rmax, cmax-4:cmax+4] = 50
    temp[centerx-4:centerx+4, centery-4:centery+4] = 2

    if plot:
        plt.imshow(temp)
        plt.plot()
        print(centerx, centery)
    # Get center of the box
    # return (centerx, centery, rmin, rmax, cmin, cmax)
    return (centerx, centery)

def get_grip_target(plot=False):
    raw_depth_map = aquire_depth_map()
    # plt.imshow(raw_depth_map)
    # plt.show()
    processed_depth_map = process_depth_map(raw_depth_map, plot)
    # centerx, centery, rmin, rmax, cmin, cmax = get_center(processed_depth_map, plot)
    centerx, centery = get_center(processed_depth_map, plot)
    #rot_angle = get_angle(raw_depth_map, np.array([centerx, centery]), rmin, rmax, cmin, cmax)

    return (centerx - processed_depth_map.shape[0]/2.0, \
            centery - processed_depth_map.shape[1]/2.0)
# Get the length from center to a corner
# l = np.linalg.norm(np.array([rmax, np.min(np.where(t[rmax, :] != 0))]) - center)

# def rotate_via_numpy(xy, radians):
#     """Use numpy to build a rotation matrix and take the dot product."""
#     x, y = xy
#     c, s = np.cos(radians), np.sin(radians)
#     j = np.matrix([[c, s], [-s, c]])
#     m = np.dot(j, [x, y])

#     return float(m.T[0]), float(m.T[1])

# # Approximation on where the corner a1 should be (yellow dot)
# b1 = rotate_via_numpy(np.array([-1, 1])*l/(2 ** .5), -1 *theta) + center
# b1 = [int(b1[0]), int(b1[1])]
# temp[b1[0]-4:b1[0]+4, b1[1]-4:b1[1]+4] = 120

# Let's see it
# plt.imshow(temp)
# plt.show()


# Experimental stu
# img_center = np.array(depth.shape)/2
# test_point = (np.array(rotate_via_numpy(center - img_center, np.pi/(-4))) + img_center).astype('int32')
# base_depth = depth[test_point[0], test_point[1]]
# box_depth = depth[center[0], center[1]]
# center_depth = depth[int(img_center[0]), int(img_center[1])]

# trans_dist = np.sqrt(np.abs(base_depth**2 - center_depth**2))
# print(trans_dist)


# np.arccos(443.0/455)* 180 / np.pi


# np.sqrt(455**2 - 443**2)

if __name__ == '__main__':
    print(get_grip_target(plot=True))
