from PIL import Image
import numpy as np

def blend_images(color_image_path, ct_image_path, output_path, alpha=0.5):
    """
    将两张RGBA图片融合在一起，生成一张新的图片。

    参数:
    - color_image_path (str): 第一张RGBA图片的路径（如color_image_3.png）
    - ct_image_path (str): 第二张RGBA图片的路径（如CT_image.png）
    - output_path (str): 输出融合后图片的路径
    - alpha (float): 融合时第一张图片的权重，范围0到1
    """
    # 检查alpha值
    if not (0.0 <= alpha <= 1.0):
        raise ValueError("Alpha值必须在0到1之间。")

    # 加载两张图片
    color_image = Image.open(color_image_path).convert("RGBA")
    ct_image = Image.open(ct_image_path).convert("RGBA")

    # 调整尺寸（如果需要）
    if color_image.size != ct_image.size:
        ct_image = ct_image.resize(color_image.size, Image.ANTIALIAS)

    # 将图像转换为NumPy数组
    color_array = np.array(color_image, dtype=np.float32)
    ct_array = np.array(ct_image, dtype=np.float32)

    # 创建一个新的数组来存储融合后的结果
    blended_array = np.zeros_like(color_array, dtype=np.float32)

    # 遍历每个像素
    for y in range(ct_array.shape[0]):
        for x in range(ct_array.shape[1]):
            # 获取当前像素的RGB值
            ct_pixel = ct_array[y, x][:3]  # 取前3个值，忽略alpha通道

            # 计算与黑色 (0, 0, 0) 的欧式距离
            distance = np.linalg.norm(ct_pixel - np.array([0, 0, 0]))

            # 判断距离是否小于阈值
            if distance < 20:
                # 如果ct_array的当前像素是黑色，alpha = 0
                alpha = 1
            else:
                # 否则alpha = 0.2
                alpha = 0.5

            # 执行加权融合
            blended_array[y, x] = alpha * color_array[y, x] + (1 - alpha) * ct_array[y, x]

    # 将融合结果转换回图像格式，并保存或显示
    blended_image = Image.fromarray(np.uint8(blended_array))
    # 确保像素值在0-255之间
    blended_array = np.clip(blended_array, 0, 255).astype(np.uint8)

    # 转换回图像
    blended_image = Image.fromarray(blended_array, 'RGBA')

    # 保存融合后的图片
    blended_image.save(output_path)

    print(f"图片已成功融合并保存到: {output_path}")

# 示例用法
if __name__ == "__main__":
    color_image_path = "imageBlend/color_image_3.png"  # 替换为你的color_image_3.png路径
    ct_image_path = "imageBlend/CT_image.png"          # 替换为你的CT_image.png路径
    output_path = "blended_image.png"       # 输出图片路径

    blend_images(color_image_path, ct_image_path, output_path, alpha=0.2)
