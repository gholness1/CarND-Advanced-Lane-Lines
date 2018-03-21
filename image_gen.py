############################################################################
# Gary Holness
# gary.holness@gmail.com
#
# Advanced Lanefinding Project
#
# For implementation of this project, I used both the code covered in the 
# course as well as approaches discussed in the review video that was available
# through a link mentioned in the course as well.
#
# My pipeline consists of the following steps
#
#  1.  Compute gradient along x and y using sobel operator and admit pixels
#      whose x and y gradient are in specified thresholds
#
#  2.  Compute color threshold using Saturation (S) and value (V) from HLS and 
#      HSV.  Admit pixels whose values are in specified thresholdsa
#
#  3.  Result of step 1 and 2 is a binary image identifying the lane lines
#
#  4.  Unwarp the image using camera calibration parameters that I created.
#      The unwarped image is used to localize the lane lines.
#
#  5.  Using histogram approach in vertical columns, localize the lane lines.
#      I use a simple convolution to do this.
#
#  6.  Run a tracker to find new location of lane lines. This approach uses
#      tracker class discussed in the overview video for this project.  The
#      tracker class examines a window around the lane line location for
#      left lane and right lane in order to save on computation.  This works
#      because position in an inhertial quantity (future value depends on
#      current value plus a delta). 
#
#  7.  Visualize the windows used for tracking
#
#  8.  fit a polygon to describe the lane lines then sample from the polygon
#      in order to draw the lane lines.    
#
#  9.  Use polygon samples for left and right lanes to color in the lane 
#      in front of the car.
#
#  10. Computer radius of curvature
#
#  Note: Abstracted all of this, once working, into a function that can be
#        called on video frames.
###

import numpy as np
import cv2
import pickle
import glob
import matplotlib.pyplot as plt
from tracker import tracker

# read the save object points and image points
dist_pickle = pickle.load(open("camera_cal/cal_dist_pickle.p","rb") )

mtx = dist_pickle["mtx"]
dist = dist_pickle["dist"]


def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    # Calculate directional gradient
    # Apply threshold


    # Apply the following steps to img
    # 1) Convert to grayscale
    # 2) Take the derivative in x or y given orient = 'x' or 'y'
    # 3) Take the absolute value of the derivative or gradient
    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    # 5) Create a mask of 1's where the scaled gradient magnitude
            # is > thresh_min and < thresh_max
    # 6) Return this mask as your binary_output image

    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    if orient== 'x':
      sobel = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    else:
      sobel = cv2.Sobel(gray, cv2.CV_64F, 0, 1)

    abs_sobel = np.absolute(sobel)

    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])] = 1

    return grad_binary

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    # Calculate gradient magnitude
    # Apply threshold


    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1)


    abs_sobel =  np.sqrt(np.multiply(sobelx,sobelx) + np.multiply(sobely,sobely))

    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    mag_binary = np.zeros_like(scaled_sobel)
    mag_binary[(scaled_sobel >= mag_thresh[0]) & (scaled_sobel <= mag_thresh[1])] = 1

    return mag_binary


def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Calculate gradient direction
    # Apply threshold

    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    sobelx = cv2.Sobel(gray, cv2.CV_64F, 1, 0,ksize=sobel_kernel)
    sobely = cv2.Sobel(gray, cv2.CV_64F, 0, 1,ksize=sobel_kernel)

    #abs_sobelx = np.sqrt(np.multiply(sobelx,sobelx))
    #abs_sobely = np.sqrt(np.multiply(sobely,sobely))

    abs_sobelx = np.absolute(sobelx)
    abs_sobely = np.absolute(sobely)
    grad_dir= np.arctan2(abs_sobely,abs_sobelx)

    dir_binary = np.zeros_like(grad_dir)

    dir_binary[(grad_dir >= thresh[0]) & (grad_dir <= thresh[1])] = 1

    return dir_binary


def color_threshold(image, sthresh=(0,255), vthresh=(0,255)):
    hls= cv2.cvtColor(image, cv2.COLOR_RGB2HLS)

    s_channel = hls[:,:,2]

    s_binary = np.zeros_like(s_channel)
    s_binary[ (s_channel >= sthresh[0]) & (s_channel <= sthresh[1]) ] = 1

    hsv= cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    v_channel= hsv[:,:,2]
    v_binary= np.zeros_like(v_channel)
    v_binary[ (v_channel >= vthresh[0]) & (v_channel <= vthresh[1])] = 1
  
    output = np.zeros_like(s_channel)
    output[ (s_binary ==1) & (v_binary == 1) ] = 1

    return output


def window_mask(width, height, img_ref, center, level):
    output = np.zeros_like(img_ref)

    output[int(img_ref.shape[0]-(level+1)*height):int(img_ref.shape[0]-level*height), \
           max(0,int(center-width)):min(int(center+width), img_ref.shape[1])] = 1
   
    return output


images = glob.glob('./test_images/test*.jpg')



#####
# Main routine for my lane finding pipeline
###
def processing_pipeline(img):
    VISUALIZE_AND_LOG= False
    DEBUG= False

    #undistort image
    imgu= cv2.undistort(img, mtx, dist, None, mtx)


    #process image and generate binary pixel
    
    preprocessImage = np.zeros_like(img[:,:,0])
    gradx= abs_sobel_thresh(img, orient='x', thresh=(12,255)) 
    grady= abs_sobel_thresh(img, orient='y', thresh=(25,255))
   
    c_binary = color_threshold(img, sthresh=(100,255), vthresh=(50,255))
    preprocessImage[ ((gradx == 1) & (grady == 1)) | (c_binary == 1) ] = 255

    if VISUALIZE_AND_LOG: 
       write_name = './test_images/tracked_thresholded' + str(idx+1) + '.jpg'
       fig = plt.figure()
       plt.imshow(preprocessImage)
       plt.show(block=False)
       cv2.imwrite(write_name, preprocessImage)


    #do the perspective transformation

    img_size = (img.shape[1], img.shape[0])
    bottom_width= 0.75  #pct of bottom polygon height
    mid_width = 0.15    #pct of middle polygon height
    height_pct= 0.65    #pct for polygon height (forward facing out into road ahead)
    bottom_trim= 0.935  #pct from top to bottom avoiding car bonnet

    src = np.float32([[img.shape[1]*(.5-mid_width/2),img.shape[0]*height_pct], \
                      [img.shape[1]*(.5+mid_width/2),img.shape[0]*height_pct], \
                      [img.shape[1]*(.5+bottom_width/2),img.shape[0]*bottom_trim], \
                      [img.shape[1]*(.5-bottom_width/2),img.shape[0]*bottom_trim]])

    offset = img_size[0]*.25

    dst = np.float32([[offset, 0], [img_size[0]-offset, 0], \
                      [img_size[0]-offset, img_size[1]], \
                      [offset ,img_size[1]]])


    #do the transform
    M = cv2.getPerspectiveTransform(src,dst)
    Minv= cv2.getPerspectiveTransform(dst, src)
    warped = cv2.warpPerspective(preprocessImage, M, img_size, flags=cv2.INTER_LINEAR)

    result = warped

    #####
    # write the unwarped binary image of lane lines to disk
    ##

    if VISUALIZE_AND_LOG:
       write_name = './test_images/tracked' + str(idx+1) + '.jpg'
       fig = plt.figure()
       plt.imshow(result)
       plt.show(block=False)
       cv2.imwrite(write_name, result)
       #plt.pause(5)

    
    #####
    # Tracking of lane lines will be done in a window around
    ###
    window_width = 25
    window_height = 80

    # establish overall class to do tracking
    curve_centers = tracker(Mywindow_width = window_width, Mywindow_height = window_height, Mymargin = 25, My_ym = 10/720, My_xm = 4/384, Mysmooth_factor = 15)	

    window_centroids = curve_centers.find_window_centroids(warped)

    # Points to draw all left and right windows
    l_points = np.zeros_like(warped)
    r_points = np.zeros_like(warped)

    # Points to find left and right lanes
    rightx = []
    leftx = []

    # Draw windows for each level
    for level in range(0, len(window_centroids)):
        # window_mask function draws window areas for each
        # each search window (at every layer)

        l_mask = window_mask(window_width,window_height, warped, window_centroids[level][0],level)
        r_mask = window_mask(window_width,window_height, warped, window_centroids[level][1],level)
        # Add center value to right, left lane points lists
        leftx.append(window_centroids[level][0])
        rightx.append(window_centroids[level][1])
        # Add graphic points from window mask to total pixels found
        l_points[(l_points == 255) | ((l_mask == 1) ) ] = 255
        r_points[(r_points == 255) | ((r_mask == 1) ) ] = 255

     # Draw the results
    template = np.array(r_points+l_points,np.uint8) # add l, r window pixels
    zero_channel = np.zeros_like(template) # create zero color channel
    template = np.array(cv2.merge((zero_channel,template,zero_channel)),np.uint8) # make window pixels green
    warpage = np.array(cv2.merge((warped,warped,warped)),np.uint8) # make original road pixels 3 color channels
    result = cv2.addWeighted(warpage, 0.2, template, 0.75, 0.0) # overlay the original road image with window results
    #fig= plt.figure()
    #plt.imshow(result)
    #plt.show(block=False)
    #plt.pause(5)


    # Fit curves to images
    # fit the lane boundaries to the left,right center positions found
    yvals = range(0,warped.shape[0])

    res_yvals = np.arange(warped.shape[0]-(window_height/2),0,-window_height)

    left_fit = np.polyfit(res_yvals, leftx, 3)
    left_fitx = left_fit[0]*yvals*yvals*yvals + left_fit[1]*yvals*yvals + left_fit[2]*yvals+left_fit[3]
    left_fitx = np.array(left_fitx,np.int32)
	
    right_fit = np.polyfit(res_yvals, rightx, 3)
    right_fitx = right_fit[0]*yvals*yvals*yvals + right_fit[1]*yvals*yvals + right_fit[2]*yvals+right_fit[3]	
    right_fitx = np.array(right_fitx,np.int32)

    # used to format everything so its ready for cv2 draw functions
    left_lane = np.array(list(zip(np.concatenate((left_fitx-window_width/2,left_fitx[::-1]+window_width/2), axis=0),np.concatenate((yvals,yvals[::-1]),axis=0))),np.int32)
    right_lane = np.array(list(zip(np.concatenate((right_fitx-window_width/2,right_fitx[::-1]+window_width/2), axis=0),np.concatenate((yvals,yvals[::-1]),axis=0))),np.int32)
    middle_marker = np.array(list(zip(np.concatenate((right_fitx-window_width/2, right_fitx[::-1]+window_width/2), axis=0),np.concatenate((yvals,yvals[::-1]),axis=0))),np.int32)

    # draw lane lines, middle curve, road background on two different blank overlays
    road = np.zeros_like(img)
    road_bkg = np.zeros_like(img)
    cv2.fillPoly(road,[left_lane],color=[255, 0, 0])
    cv2.fillPoly(road,[right_lane],color=[0, 0, 255])
    cv2.fillPoly(road_bkg,[left_lane],color=[255, 255, 255])
    cv2.fillPoly(road_bkg,[right_lane],color=[255, 255, 255])

    if DEBUG:
       print("left_lane");
       print(left_lane)
       print("right_lane")
       print(right_lane)

    if VISUALIZE_AND_LOG:
       fig= plt.figure()
       plt.imshow(result)
       plt.show(block=False)
       write_name = './test_images/road_warped_layers'+str(idx+1)+'.jpg'
       cv2.imwrite(write_name, result)
       #plt.pause(5)


    pts = np.hstack((left_lane, right_lane))
    #print(pts)

    # Draw the lane onto the warped blank image

    #####
    # create N x 2 matrix of polygon points for lane by
    # tracing down the left lane line and up the right
    # lane line.
    ###
    roadPolygonL= np.column_stack((left_fitx[0:] ,yvals[0:]))
    roadPolygonR= np.column_stack( (right_fitx[::-1], yvals[::-1]) )
    roadPolygon= np.vstack((roadPolygonL, roadPolygonR))

    if DEBUG:
       print("shape(roadPolygon)= ",roadPolygon.shape)
       print(roadPolygon)

    result= cv2.fillPoly(road, [roadPolygon], color=(0,255,0))
    result= cv2.fillPoly(road_bkg, [roadPolygon], color=(0,255,0))


    road_warped = cv2.warpPerspective(road,Minv,img_size,flags=cv2.INTER_LINEAR)
    road_warped_bkg = cv2.warpPerspective(road_bkg,Minv,img_size,flags=cv2.INTER_LINEAR)

    base = cv2.addWeighted(img, 1.0, road_warped_bkg, -0.6, 0.0) 
    result = cv2.addWeighted(base, 1.0, road_warped, 0.9, 0.0)


    ym_per_pix = curve_centers.ym_per_pix # meters/pixel in y dimension
    xm_per_pix = curve_centers.xm_per_pix # meters/pixel in x dimension

    curve_fit_cr = np.polyfit(np.array(res_yvals,np.float32)*ym_per_pix, np.array(leftx,np.float32)*xm_per_pix, 2)
    curverad = ((1 + (2*curve_fit_cr[0]*yvals[1]*ym_per_pix + curve_fit_cr[1])**2)**1.5) /np.absolute(2*curve_fit_cr[0])

    # calculate the offset of the car on the road
    camera_center = (left_fitx[-1] + right_fitx[-1])/2
    center_diff = (camera_center-warped.shape[1]/2)*xm_per_pix
    side_pos = 'left'
    if center_diff <= 0:
       side_pos = 'right'

    cv2.putText(result,'Radius of Curvature = '+str(round(curverad,3))+'(m)',(50,50) , cv2.FONT_HERSHEY_SIMPLEX, 1,(255, 255, 255),2)
    cv2.putText(result,'Vehicle is '+str(abs(round(center_diff,3)))+'m '+side_pos+' of center',(50,100) , cv2.FONT_HERSHEY_SIMPLEX, 1,(255, 255, 255),2)


    if VISUALIZE_AND_LOG: 
       write_name = './test_images/road_warped'+str(idx+1)+'.jpg'
       fig = plt.figure()
       plt.imshow(result)
       plt.show(block=False)
       result = cv2.cvtColor(result,cv2.COLOR_RGB2BGR);
       cv2.imwrite(write_name, result)
       plt.pause(5)

    return result



#####
# Main processing loop loading images and calling
# the processing routine for lane finding
###
for idx, fname in enumerate(images):
    #read in image
    img = cv2.imread(fname)
    img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    output= processing_pipeline(img)


from moviepy.editor import VideoFileClip

#video_input= './project_video.mp4'
#video_input= './challenge_video.mp4'
video_input= './harder_challenge_video.mp4'

#video_output= './myvideo.mp4'
#video_output= './myvideo_challenge.mp4'
video_output= './myvideo_harder_challenge.mp4'

video_clip = VideoFileClip(video_input)

processed_video= video_clip.fl_image(processing_pipeline)
processed_video.write_videofile(video_output, audio=False)
