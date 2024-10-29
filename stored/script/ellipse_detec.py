import cv2
import numpy as np

def evaluate_fit_quality(contour, ellipse):
    center, axes, angle = ellipse
    center = np.array(center)
    angle_rad = np.deg2rad(angle)
    cos_angle = np.cos(angle_rad)
    sin_angle = np.sin(angle_rad)
    a = axes[0] / 2
    b = axes[1] / 2
    total_distance = 0
    for point in contour:
        x, y = point[0]
        dx = x - center[0]
        dy = y - center[1]
        x_rot = dx * cos_angle + dy * sin_angle
        y_rot = -dx * sin_angle + dy * cos_angle
        distance = ((x_rot / a) ** 2 + (y_rot / b) ** 2) - 1
        total_distance += abs(distance)
    mean_distance = total_distance / len(contour) / len(contour)
    return mean_distance

# 读取图像
img = cv2.imread('D:\data\project\VisualStudio\C_Azure\Sample_image\color_image_1.png')
img = cv2.imread('D:\data\project\VisualStudio\C_Azure\captures2\captures\color_image_2.png')
#cv2.imshow('Original Image', img)
#cv2.waitKey(0)

# 转换为灰度图像
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
#cv2.imshow('Grayscale Image', gray)
#cv2.waitKey(0)

# 高斯模糊（减少噪声）
blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#cv2.imshow('Gaussian Blurred Image', blurred)
#cv2.waitKey(0)

# 自适应阈值分割
adaptive_thresh = cv2.adaptiveThreshold(
    blurred, 255,
    cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
    cv2.THRESH_BINARY_INV, 11, 2)
#cv2.imshow('Adaptive Thresholding', adaptive_thresh)
#cv2.waitKey(0)

# 形态学开运算（消除小噪声）
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
morph = adaptive_thresh
#morph = cv2.morphologyEx(adaptive_thresh, cv2.MORPH_OPEN, kernel)
#cv2.imshow('Morphological Opening', morph)
#cv2.waitKey(0)

# 形态学闭运算（填充内部空洞）
morph = cv2.morphologyEx(morph, cv2.MORPH_CLOSE, kernel)
cv2.imshow('Morphological Closing', cv2.resize(morph, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
cv2.waitKey(0)

# 寻找轮廓
contours, hierarchy = cv2.findContours(
    morph, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 绘制轮廓
contour_img = img.copy()
cv2.drawContours(contour_img, contours, -1, (255, 0, 0), 2)
# cv2.imshow('Contours', contour_img)
# cv2.waitKey(0)

# 设置长轴长度的最大值
max_axis = 120  # 根据您的需求设置
max_e = 0.85
fit_quality_threshold = 0.002

# 拟合椭圆并绘制
ellipse_img = img.copy()
for cnt in contours:
    area = cv2.contourArea(cnt)
    if area > 500 and len(cnt) >= 5:
        try:
            ellipse = cv2.fitEllipse(cnt)
            center, axes, angle = ellipse
            # 获取轴长度
            (axis1, axis2) = axes
            if axis1 < axis2:
                axis1, axis2 = axis2, axis1
            e = np.sqrt(1 - (axis2 ** 2) / (axis1 ** 2))
            print(e, axis1)
            # 检查轴长度是否为正
            if axes[0] > 0 and axes[1] > 0:
                # 应用长轴长度的最大值限制
                mean_distance = evaluate_fit_quality(cnt, ellipse)
                print(mean_distance)
                if axis1 <= max_axis and e < max_e and mean_distance <= fit_quality_threshold:
                    print("##############", center)
                    cv2.ellipse(ellipse_img, ellipse, (0, 255, 0), 2)
        except cv2.error as e:
            print("Error fitting ellipse:", e)

cv2.imshow('Detected Ellipses with Max Major Axis Length Constraint', cv2.resize(ellipse_img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_AREA))
cv2.waitKey(0)
cv2.destroyAllWindows()
