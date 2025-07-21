import cv2
import numpy as np
import yaml

filename = "rtabmap2"

# === Step 1: 讀取地圖與設定 ===
map_img = cv2.imread(f"{filename}.pgm", cv2.IMREAD_UNCHANGED)  # 原始灰階值
scale = 4
# map_img = cv2.resize(map_img, (map_img.shape[1]*scale, map_img.shape[0]*scale), interpolation=cv2.INTER_NEAREST)
with open(f"{filename}.yaml", 'r') as f:
    map_info = yaml.safe_load(f)

resolution = map_info['resolution']
origin = map_info['origin']  # [x, y, theta]

# === Step 2: 建立彩色版本地圖 ===
height, width = map_img.shape
colored_map = np.zeros((height, width, 3), dtype=np.uint8)

# 把未知區域（像素值==0）變成灰色
colored_map[map_img == 0] = [127, 127, 127]      # 灰
# 把可通行區域（255）變成白色
colored_map[map_img == 255] = [255, 255, 255]    # 白
# 把障礙區（其他值）變成黑色
colored_map[(map_img > 0) & (map_img < 255)] = [0, 0, 0]  # 黑

# === Step 3: 計算原點在圖片中的像素位置 ===
origin_x, origin_y, _ = origin
origin_px = int(-origin_x / resolution)
origin_py = int(height + (origin_y / resolution))  # y 軸需反轉

# 確保原點在圖片範圍內再畫
if 0 <= origin_px < width and 0 <= origin_py < height:
    cv2.drawMarker(
        colored_map,
        position=(origin_px, origin_py),
        color=(0, 0, 255),  # 紅色
        markerType=cv2.MARKER_CROSS,
        markerSize=20,
        thickness=2
    )
    cv2.putText(
        colored_map,
        'Origin',
        (origin_px + 10, origin_py - 10),
        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
        fontScale=0.5,
        color=(0, 0, 255),
        thickness=1
    )

# === Step 4: 顯示或儲存結果 ===
cv2.imshow("Colored Map with Origin", colored_map)
cv2.imwrite("map_colored.png", colored_map)
cv2.waitKey(0)
cv2.destroyAllWindows()
