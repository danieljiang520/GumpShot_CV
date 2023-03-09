import cv2
import numpy as np
# Stacks multiple images into 1 large image, useful for comparing and testing.
def stackImages(scale,imgArray,lables=[]):
    sizeW= imgArray[0][0].shape[1]
    sizeH = imgArray[0][0].shape[0]
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range ( 0, rows):
            for y in range(0, cols):
                imgArray[x][y] = cv2.resize(imgArray[x][y], (sizeW, sizeH), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y]= cv2.cvtColor( imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank]*rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
            hor_con[x] = np.concatenate(imgArray[x])
        ver = np.vstack(hor)
        ver_con = np.concatenate(hor)
    else:
        for x in range(0, rows):
            imgArray[x] = cv2.resize(imgArray[x], (sizeW, sizeH), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor= np.hstack(imgArray)
        hor_con= np.concatenate(imgArray)
        ver = hor
    if len(lables) != 0:
        eachImgWidth= int(ver.shape[1] / cols)
        eachImgHeight = int(ver.shape[0] / rows)
        print(eachImgHeight)
        for d in range(0, rows):
            for c in range (0,cols):
                cv2.rectangle(ver,(c*eachImgWidth,eachImgHeight*d),(c*eachImgWidth+len(lables[d][c])*13+27,30+eachImgHeight*d),(255,255,255),cv2.FILLED)
                cv2.putText(ver,lables[d][c],(eachImgWidth*c+10,eachImgHeight*d+20),cv2.FONT_HERSHEY_COMPLEX,0.7,(255,0,255),2)
    return ver


cap = cv2.VideoCapture(0)
print("capturing")
hsvRange=[[[170,170,80],[180,255,255]],[[0,170,80],[10,255,255]]]
while cap.isOpened():
    ret, img = cap.read()
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(hsv,np.array([hsvRange[0][0]]),np.array([hsvRange[0][1]]))
    if(len(hsvRange)>1):
        for x in hsvRange[1:]:
            mask2=cv2.inRange(hsv,np.array([x[0]]),np.array([x[1]]))
            mask = cv2.bitwise_or(mask,mask2)
    #cv2.HoughCircles(input, method, dp, spacing, None, param1(internal canny detector), param2(confermation thresh), minRadius, maxRadius)
    circles=cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1.2, img.shape[0], None,100,25, int(img.shape[0]/40), int(img.shape[1]/5))#, int(img.shape[0]/10), int(img.shape[1]/2)

    if circles is not None:#if circles is not empty
        circles=np.round(circles[0,:]).astype("int")#formatting line
        for(a,b,r) in circles:#iterates through all circles found in crop
            cv2.circle(img,(a,b),r, (255,0,0),7)#draws a circle
                        
    imgStack=stackImages(0.3, ([img],[mask]))
    cv2.imshow('img',imgStack)
    k = cv2.waitKey(27) & 0xff

    if k==ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print("end"); 