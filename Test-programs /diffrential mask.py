import cv2
import numpy as np

circle = np.zeros((800,800), dtype="uint8")
Csys=(400,420) #x,y coordinates 0,0 i venstre top
Dia=(290)
cv2.circle(circle, Csys,Dia,255,-1)



mask = cv2.imread('mask clean.jpg' , cv2.IMREAD_GRAYSCALE)
target = cv2.imread('target2.jpg' , cv2.IMREAD_GRAYSCALE)

maskINV = cv2.bitwise_not(mask)
targetINV = cv2.bitwise_not(target)

#cv2.imshow("mask",maskINV)
#cv2.imshow("target", targetINV)

diffrens = maskINV-targetINV
#overlay = cv2.addWeighted(circle,1, diffrens, 0,0)

B=10
thresh1, thresh= cv2.threshold(diffrens,B,255,cv2.THRESH_BINARY)
thresh2, thresh2= cv2.threshold(target,B,255,cv2.THRESH_BINARY)


cv2.imshow("thresh", thresh)
cv2.imshow("target", thresh2)


cv2.waitKey(0)
cv2.destroyAllWindows()

