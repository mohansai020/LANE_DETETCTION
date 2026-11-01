# LANE_DETETCTION
RGB image -> gray (eq. 8: Y = (5R + 9G + 2B) / 16)
          -> Canny: Gaussian 5x5 -> Sobel + square root -> non-maximum suppression -> hysteresis
          -> Hough transform with a CORDIC rotator (101 angles, 1.79 deg steps), bottom half only
          -> strongest line in the left and right lane regions
          -> lanes drawn on a 640x480 @ 60 Hz VGA picture
