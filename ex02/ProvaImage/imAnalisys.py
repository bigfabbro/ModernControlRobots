import cv2
import numpy as np
import imutils

class imageAnalisys:

    def positionShapeColour(self, filename, colorToSearch, shapeToSearch):
        image = cv2.imread(filename) # read the image
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB) # transformation in RGB
        center = ()
        find = False
        # depending on the color that we want to recognize bounds are fixed
        if colorToSearch == "red":
            lower_bound = np.array([170, 0, 0])
            upper_bound = np.array([255, 180, 180])
        elif colorToSearch == "green":
            lower_bound = np.array([0, 170, 0])
            upper_bound = np.array([100, 255, 100])
        elif colorToSearch == "blue":
            lower_bound = np.array([40, 40, 80])
            upper_bound = np.array([85, 85, 255])
        elif colorToSearch == "yellow":
            lower_bound = np.array([100, 110, 0])
            upper_bound = np.array([200, 200, 90])
        # all the pixels with RGB values not in the range [lower_bound, upper_bound] are fixed to zero
        mask = cv2.inRange(image_rgb, lower_bound, upper_bound)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        # find the contours of the figures
        cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        # if there is at least one shape
        if len(cnts)>0:
            i = 0
            num_cnts = len(cnts)
            while i < num_cnts:
                # approximate a polygonal curve
                approx = cv2.approxPolyDP(cnts[i], 0.01 * cv2.arcLength(cnts[i], True), True)
                # if the number of vertices of the polygonal curve is lower than 8 probably we haven't a circle in our
                # image, in the other way if the number of vertices is higher than 4, in our case, probably we haven't
                # a rectangle in our image.
                if (shapeToSearch == "Circle" and len(approx) < 8) or (shapeToSearch == "Rectangle" and len(approx) > 4):
                    # deleting the shape from our set
                    cnts = np.delete(cnts,i,0)
                    num_cnts = len(cnts)
                else:
                    i = i + 1
            # if we have at least one shape in our set
            if len(cnts)>0:
                # find the shape with the max area, so the closer at the camera
                m = max(cnts, key=cv2.contourArea)
                M = cv2.moments(m)
                # calculate the centroid of the shape
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                find = True
                # draw the centroid on the image
                cv2.circle(image, center, 5, (0, 255, 255), -1)
            # display of the images --> we have to delete this part in the final version
            cv2.imshow("first image", image)
            cv2.imshow("color detection", mask)
            # press 0 to close the windows
            cv2.waitKey(0)
            cv2.destroyAllWindows()

        return find, center

an = imageAnalisys()
# find the closer object whit the shape that is indicated in the second parameter and with the color indicated in
# first parameter. It returns a boolean value and a tuple with the position of the object.
find, position = an.positionShapeColour("scena1.png", "red", "Circle")
if find:
    print(position)