import numpy as np
import cv2

#####
# tracker class
##
class tracker():
    # Constructor
    def __init__(self, Mywindow_width, Mywindow_height, Mymargin, My_ym = 1, My_xm = 1, Mysmooth_factor = 15):
        # window of previous observations for left/right center values 
        self.center_obs = []

        # the window pixel width of the center values, used to count pixels inside center windows to determine curve values
        self.window_width = Mywindow_width

        # the window pixel height of the center values, used to count pixels inside center windows to determine curve values
        # breaks the image into vertical levels
        self.window_height = Mywindow_height

        self.margin = Mymargin

        self.ym_per_pix = My_ym # meters/pixel in vertical axis

        self.xm_per_pix = My_xm # meters/pixel in horizontal axis

        self.smooth_factor = Mysmooth_factor

        # main tracking function for finding, storing lane segment positions
    def find_window_centroids(self, warped):

        window_width = self.window_width
        window_height = self.window_height
        margin = self.margin

        window_centroids = [] # Store l, r window centroid positions each level
        window = np.ones(window_width) # Create window template for use with convolutions

        #####
        # Sum quarter bottom of image to get slice
        # this is the histogram alluded to in lesson module made by squashing
        # a 1-D signal 
        #
        # Do this for left and right plus/minus half width
        ###

        #sum of binary points for left side of centroid
        l_sum = np.sum(warped[int(3*warped.shape[0]/4):,:int(warped.shape[1]/2)], axis=0)

        # np.colvolve is the convolution from center of window
        l_center = np.argmax(np.convolve(window,l_sum))-window_width/2

        #sum of binary points for right side of centroid 
        r_sum = np.sum(warped[int(3*warped.shape[0]/4):,int(warped.shape[1]/2):], axis=0)

        # np.colvolve is the convolution from center of window for right side
        r_center = np.argmax(np.convolve(window,r_sum))-window_width/2+int(warped.shape[1]/2)

        # Add determinations for first layer
        # record the convolution for histogram (sum) for half window width to
        # the left and right of centroid

        window_centroids.append((l_center,r_center))

        # iterate through the layers and search for the
        # location associated with maximum pixels.  These
        # are the strongest lane lines. We have as many layers
        # as there are height of image divided by search window
        # heights.
        #
        # take vertical colums in image corresponding to the window
        # hight and convolve in order to count pixels. Note that
        # an array is constructed from vertical slice

        for level in range(1,(int)(warped.shape[0]/window_height)):
            # Convolve the window into the verticle slice of the image
            image_layer = np.sum(warped[int(warped.shape[0]-(level+1)*window_height):int(warped.shape[0]-level*window_height),:], axis=0)
            conv_signal = np.convolve(window, image_layer)

            # Use past left center as ref. to find best left centroid
            # window_width/2 as offset as convolutional signal ref. is at right side of window, not center
            offset = window_width/2
            l_min_index = int(max(l_center+offset-margin, 0))
            l_max_index = int(min(l_center+offset+margin, warped.shape[1]))
            l_center = np.argmax(conv_signal[l_min_index:l_max_index])+l_min_index-offset
            # and again for right side
            r_min_index = int(max(r_center+offset-margin, 0))
            r_max_index = int(min(r_center+offset+margin, warped.shape[1]))
            r_center = np.argmax(conv_signal[r_min_index:r_max_index])+r_min_index-offset

            # record the position of the new maximum 
            # to list of centroids for the current layer
            window_centroids.append((l_center, r_center))

        self.center_obs.append(window_centroids)
        # return averaged values of line centers

        # try median instead of average (not used)
        #return np.median(self.center_obs[-self.smooth_factor:], axis = 0)   

        return np.average(self.center_obs[-self.smooth_factor:], axis = 0)   
