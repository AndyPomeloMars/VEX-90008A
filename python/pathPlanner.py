# V5RC_BACKGROUND = "v5rc-background.png"
# V5RC_MAP = "v5rc-pushback.png"

import cv2
import numpy as np

# [新功能] 添加一个专门用于保存数组的函数
def save_grid_to_txt(grid, filename="field_grid.txt"):
    """
    将一个二维Numpy数组保存到指定的文本文件中。

    参数:
        grid (numpy.ndarray): 要保存的二维数组。
        filename (str): 输出的文件名。
    """
    print(f"\n正在将数组保存到文件 '{filename}'...")
    try:
        # 使用 numpy.savetxt 函数高效地保存数组
        # fmt='%d' 表示以整数格式保存
        # delimiter=',' 表示使用逗号作为数字之间的分隔符（生成CSV格式）
        np.savetxt(filename, grid, fmt='%d', delimiter=',')
        print(f"数组成功保存！您现在可以打开 '{filename}' 查看。")
    except Exception as e:
        print(f"错误：保存文件时遇到问题 - {e}")

def isolate_objects_and_create_grid(background_path, foreground_path, threshold_value=30):
    """
    通过比较背景和前景图片，分离出物件，并生成一个二值化的二维数组。

    在这个数组中：
    - 0 代表背景
    - 1 代表检测到的物件 (障碍物、得分球等)

    参数:
        background_path (str): 纯背景图片的路径。
        foreground_path (str): 带有物件的前景图片的路径。
        threshold_value (int): 颜色差异的敏感度阈值 (0-255)。

    返回:
        numpy.ndarray: 标记了物件位置的二维数组。如果图片加载失败，则返回 None。
    """
    print("--- 开始物件分离与网格生成 ---")
    
    # 1. 加载两张图片
    print(f"加载背景图片: '{background_path}'")
    background_img = cv2.imread(background_path)
    print(f"加载前景图片: '{foreground_path}'")
    foreground_img = cv2.imread(foreground_path)

    if background_img is None or foreground_img is None:
        print("错误：无法加载一张或两张图片。请检查文件路径。")
        return None
    if background_img.shape != foreground_img.shape:
        print("错误：两张图片的尺寸不匹配！")
        return None
    
    print("图片加载成功，尺寸匹配。")

    # 2. 计算两张图片的绝对差异
    difference_img = cv2.absdiff(background_img, foreground_img)
    print("已计算图片差异...")

    # 3. 将差异图片转换为灰度图
    gray_difference = cv2.cvtColor(difference_img, cv2.COLOR_BGR2GRAY)
    print("已将差异图转换为灰度图...")

    # 4. 应用阈值，生成“掩码” (Mask)
    ret, mask = cv2.threshold(gray_difference, threshold_value, 255, cv2.THRESH_BINARY)
    print(f"已使用阈值 {threshold_value} 生成物件掩码...")

    # 5. 清理掩码上的噪点
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=2)
    print("已对掩码进行清理和强化...")

    # 6. 将掩码转换为最终的二维数组 (0 和 1)
    final_grid = (mask / 255).astype(int)
    print("已生成最终的二维数组！")

    # --- 可视化 ---
    print("\n正在显示处理过程的可视化结果... 按任意键退出。")
    
    highlight_overlay = np.zeros_like(foreground_img)
    highlight_overlay[mask == 255] = [0, 0, 255]
    highlighted_result = cv2.addWeighted(foreground_img, 1.0, highlight_overlay, 0.5, 0)

    # 缩小图片以便于在屏幕上显示
    display_foreground = cv2.resize(foreground_img, (600, 600))
    display_difference = cv2.resize(difference_img, (600, 600))
    display_mask = cv2.resize(mask, (600, 600))
    display_result = cv2.resize(highlighted_result, (600, 600))
    
    # [ 关键修正 ] 将2D的mask图像转换为3D的BGR图像
    # 这样它才能和彩色的 display_result 图像进行拼接
    display_mask_3d = cv2.cvtColor(display_mask, cv2.COLOR_GRAY2BGR)

    # 添加标题
    cv2.putText(display_foreground, "Foreground", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.putText(display_difference, "Difference", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.putText(display_mask_3d, "Mask (Detected Objects)", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)
    cv2.putText(display_result, "Final Result", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

    # 将四张图片拼接在一起显示
    top_row = np.hstack([display_foreground, display_difference])
    # [ 关键修正 ] 使用转换后的3D mask图像进行拼接
    bottom_row = np.hstack([display_mask_3d, display_result])
    combined_display = np.vstack([top_row, bottom_row])

    cv2.imshow("Object Detection via Background Subtraction", combined_display)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    
    return final_grid

# --- 主程序 ---
if __name__ == '__main__':
    # 您的图片文件名
    BACKGROUND_IMAGE_FILE = 'v5rc-background.png'
    FOREGROUND_IMAGE_FILE = 'v5rc-pushback.png'

    object_grid = isolate_objects_and_create_grid(BACKGROUND_IMAGE_FILE, FOREGROUND_IMAGE_FILE)

    if object_grid is not None:
        print("\n--- 最终数组验证 ---")
        print(f"生成的二维数组形状 (行, 列): {object_grid.shape}")
        
        num_object_pixels = np.sum(object_grid)
        total_pixels = object_grid.shape[0] * object_grid.shape[1]
        coverage_percentage = (num_object_pixels / total_pixels) * 100
        
        print(f"在 {total_pixels} 个总像素中，检测到 {num_object_pixels} 个物件像素点。")
        print(f"物件覆盖了场地的 {coverage_percentage:.2f}%。")

        # [新功能] 调用保存函数，将生成的数组存入TXT文件
        save_grid_to_txt(object_grid, filename="field_grid.txt")