import cv2 as cv
import numpy as np
 
# 读入图片
img = np.zeros((200, 600), np.uint8)
# 调用cv.putText()添加文字
text1 = "Initializing,  "
text2="current best guess:  "
text3="x=1.0, y=-4.5, yaw= 180.0"
AddText = img.copy()
cv.putText(AddText, text1, (20, 60), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255), 1)
cv.putText(AddText, text2, (20, 100), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255), 1)
cv.putText(AddText, text3, (20, 140), cv.FONT_HERSHEY_COMPLEX, 1.0, (255,255,255), 1)
AddText1 = img.copy()
text4="Pose tracking"
cv.putText(AddText1, text4, (20, 100), cv.FONT_HERSHEY_COMPLEX, 2, (255,255,255), 2)

# 将原图片和添加文字后的图片拼接起来
res = np.hstack([AddText, AddText1])
 
# 显示拼接后的图片
cv.imshow('text', res)
cv.waitKey()
cv.destroyAllWindows()
