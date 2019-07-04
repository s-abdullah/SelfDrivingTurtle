import cv2,  numpy


cv2.namedWindow("window", 1)
cv2.namedWindow("window2", 1)

im2 = cv2.imread("left.png")
im22 = cv2.imread("right.png")

blur = cv2.blur(im2,(5,5))
blur2 = cv2.blur(im22,(5,5))

cv2.imwrite("rBlur.png", blur2)
cv2.imwrite("lBlur.png", blur)


# cv2.imshow("window", cv2.flip(blur, 1))
# cv2.imshow("window2", blur2)

cv2.waitKey(0)
cv2.destroyAllWindows()