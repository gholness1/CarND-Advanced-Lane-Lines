## Lane Finding Project

---

**Gary Holness**

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./test_images/test1.jpg "Original Image1"
[image2]: ./test_images/tracked_thresholded1.jpg "Binary Image Thresholded"
[image3]: ./test_images/tracked1.jpg "Unwarped Road"
[image4]: ./test_images/road_warped_layers1.jpg  "Layers for lane lines"
[image5]: ./test_images/road_warped1.jpg "Unwarped road with lanes mapped"
[image6]: ./camera_cal/calibration1.jpg "original distorted image"
[image7]: ./camera_cal/calibration1_undist.jpg "calibrated unwarped image"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup 

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  


### Camera Calibration

#### 1. This was done in python program file `cam_cal.py`

I prepare the object points (line 11-13) using mesh grid to create ground truth points for the grid locations in
the image of the checkerboard.  The chessboard assumes the z-axis is zero meaning the x-y plane is visible in the
images.  I tectect corners using OpenCV2's `cv2.findChessboardCorners()` function on a grayscale version of the
color image.

I populate an array with these image points as well as the object points (lines 31-40) and then validate by drawing
connections between corner points.   Finally, I call OpenCV2's calibration API `cv2.calibrateCamera()` which
returns the parameter matrix for the calibration.  I test out the calibration by undistorting the original image
and verify that it works.

![original image][image6]
![unwarped image][image7]

### Pipeline (single images)

My lane finding pipeline is in the program file `image_gen.py` where I implement the lanefinding pipeline and
`tracker.py` where i implement the tracker for lane lines.  For implementation of this project, I relied
on examples and code from the course content as well as the tutorial/help video linked by the course
content.

My pipeline consists of the following steps

1.  Compute gradient along x and y using sobel operator and admit pixels
    whose x and y gradient are in specified thresholds

2.  Compute color threshold using Saturation (S) and value (V) from HLS and
    HSV.  Admit pixels whose values are in specified thresholdsa

3.  Result of step 1 and 2 is a binary image identifying the lane lines

4.   Unwarp the image using camera calibration parameters that I created.
     The unwarped image is used to localize the lane lines.

5.  Using histogram approach in vertical columns, localize the lane lines.
    I use a simple convolution to do this.

6.  Run a tracker to find new location of lane lines. This approach uses
    tracker class discussed in the overview video for this project.  The
    tracker class examines a window around the lane line location for
    left lane and right lane in order to save on computation.  This works
    because position in an inhertial quantity (future value depends on
    current value plus a delta).

7.  Visualize the windows used for tracking

8.  fit a polygon to describe the lane lines then sample from the polygon
    in order to draw the lane lines.

9.  Use polygon samples for left and right lanes to color in the lane
    in front of the car.

10. Computer radius of curvature

#### 1. Provide an example of original image

I begin with an original image containing distortion.
![original image][image1]

#### 2. I use multiple transformations based on color to threshold the image and identify lane lines

My approach (lines 62-90, 136-152 image_gen.py) uses Saturation(S) from HLS colorspace and Value from HSV
colorspace.  I combine color thresholding with gradients along x and y using the sobel operator.  This
results in a very nice binary image where lane lines are clearly visibile

![thresholded binary image][image2]


#### 3. I unwarp the binary image to make it easier to identify the lane lines
Using the calibration information stored in a pickle file, I load it and perform unwarping (line 175, `image_gen.py`).
An example of an undistorted image pertains to the binary image identifying the lane lines.

![unwarped image][image3]

#### 4. I identify the lane lines using a histogram approach in vertical columns.

The histogram approach sums vertical columns in the binary image looking for peaks.  Because lane lines are white
in the binary image, the highest (lines 29-62, tracker.py) sums (peaks) correspond to locations wehre there are lane lines.
I employ convolution to do the localization, that is finding where alogn the sums the peak occurs.  I used 
a windowed approach to localizing the line (lines 64-92, tracker.py).  

A version of the image identifying the windowed search and identification of lane lines is shown.

![windowed search for lane lines][image4]

#### 4.  I then draw a filled polygon identifying the lane and apply inverse tranform.

Once I have the lines for the left and right lane lines, I fit them to a polynomial (lines 281-293).
Fitting polynomials gives me a parametric form from which I can sample (lines 288, 292) for use in
drawing the lane lines as well as filling a polygon to identify the lane.  To identify the lane,
i traverse town the left lane line's points and up the right lane line's points (line 328-342).
After drawing the polygon on the unwarped image, I rewarp it to get the lane identified in
the original image (lines 345-346).

The resulting performance is pretty good.

![identified lane and lane lines][image5]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

[![video result](http://img.youtube.com/vi/2fRMQcLh4tE/0.jpg)](http://www.youtube.com/watch?v=2fRMQcLh4tE "video result")

---

### Discussion

#### 1. Briefly discuss any problems 

This project presented numerous issues.  Honestly the biggest was getting python to work. I implemented a number
of different thresholding methods because I wasn't sure which I would need.  The Sobel gradients along x and y
along with color worked well for me.   The review/tutorial video was immensely helpful.  Problems I encountered
had more to do with Python syntax.  I did find I had to spend alot of time tuning the threshold parameters.
