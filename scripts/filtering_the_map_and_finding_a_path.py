import cv2

# loading the frame
frame = cv2.cvtColor(cv2.imread('/home/zack/v1_robotws/src/trying_again/map_19.jpg'),cv2.COLOR_BGR2GRAY)
# thresholding the frame so if the value in a cell is 30 or above it is true showing its okat to go in
ret,thresh = cv2.threshold(frame,30,255,cv2.THRESH_BINARY)
# kernel for the mask
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
# closeing the image to remove anything that might be nothing
mask = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
# showing the masked image and the original image
cv2.imshow('regular image',frame)
cv2.imshow('masked image',mask)

while(1):
    if cv2.waitKey(1) & 0xFF == 27:
        break
cv2.destroyAllWindows()
# finding the contours in the mask
contours, hierarchy = cv2.findContours(mask,cv2.RETR_LIST,cv2.CHAIN_APPROX_NONE)