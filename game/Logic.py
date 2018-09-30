class Play():
    def CenterOnGreenBall():
        #grab the reference to the webcam
        camera = cv2.VideoCapture(1)
        precision = 20 # How far from the center we give as good

        while True:
            # grab the current frame
            (grabbed, frame) = camera.read()
            xc = frame.size[0]/2 # Horizontal center of the image
            center  =  LocateBallCenter(frame)

            if center is not None:
                x,y = center
                if (x < xc + precision) and  (x > xc - precision):
                    error = x - xc
                    break

        #print('Distance from center:',error)
        return error