import sensor, image, time
from machine import UART
from pyb import LED

# -------------------------- 配置参数 --------------------------
# 固定ROI宽高（面积不变）
ROI_WIDTH = 77
ROI_HEIGHT = 55
GRID_SIZE = 11

# 4个指定的ROI起始坐标 (x, y) 【左上、右上、左下、右下】
ROI_COORDINATES = [(68,124), (142,123), (68,180), (142,181)]

#箱子，目的地，墙，英雄
COLOR_MAP = {
    '3': (65, 100, -60, -24, 9, 127),
    '4': (0, 100, 27, 127, -68, 127),
    '1': (40, 100, -30, 30, -71, 4),
    '2': (0, 100, -128, -22, -128, 60)
}

DETECT_THRESHOLD = {
    '1': {'pixels': 40, 'area': 40},
    'default': {'pixels': 20, 'area': 20}
}

# -------------------------- 串口配置 --------------------------
# OpenMV H7 可用 UART:
#   UART(1): TX=P1,  RX=P0  (默认引脚)
#   UART(2): TX=P4,  RX=P5
#   UART(3): TX=P9,  RX=P10
#   UART(4): TX=PA10, RX=PA9 (USB虚拟串口，不推荐)
#
# 【重要】请根据实际接线修改 UART_ID！
# 如果 RT1064 的 C16/C17 连接到 OpenMV 的 P0/P1，则用 UART_ID = 1
# 如果连接到 P4/P5，则用 UART_ID = 2
# 如果连接到 P9/P10，则用 UART_ID = 3
UART_ID = 1
UART_BAUD = 115200
led_blue = LED(3)

# 初始化 UART，指定引脚（根据 UART_ID 自动选择）
if UART_ID == 1:
    uart = UART(UART_ID, UART_BAUD, tx=1, rx=0)
elif UART_ID == 2:
    uart = UART(UART_ID, UART_BAUD, tx=4, rx=5)
elif UART_ID == 3:
    uart = UART(UART_ID, UART_BAUD, tx=9, rx=10)
else:
    uart = UART(UART_ID, UART_BAUD)
# -------------------------------------------------------------

# 初始化摄像头
def init_camera():
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time=2000)
    sensor.set_auto_gain(False)
    sensor.set_auto_whitebal(False)
    grid_cols = ROI_WIDTH // GRID_SIZE
    grid_rows = ROI_HEIGHT // GRID_SIZE
    return grid_cols, grid_rows

# 单个网格检测
def detect_grid_blob(img, grid_roi):
    for char, threshold in COLOR_MAP.items():
        pix_thresh = DETECT_THRESHOLD['1']['pixels'] if char == '1' else DETECT_THRESHOLD['default']['pixels']
        area_thresh = DETECT_THRESHOLD['1']['area'] if char == '1' else DETECT_THRESHOLD['default']['area']
        blobs = img.find_blobs([threshold], roi=grid_roi, pixels_threshold=pix_thresh, area_threshold=area_thresh, merge=False, lab=True)
        if blobs:
            return char
    return '0'

# 检测单个ROI区域
def detect_single_roi(grid_cols, grid_rows, roi_x, roi_y):
    img = sensor.snapshot()
    char_map = []
    for row in range(grid_rows):
        row_str = ""
        for col in range(grid_cols):
            grid_x = roi_x + col * GRID_SIZE
            grid_y = roi_y + row * GRID_SIZE
            grid_char = detect_grid_blob(img, (grid_x, grid_y, GRID_SIZE, GRID_SIZE))
            row_str += grid_char
        char_map.append(row_str)
    return ''.join(char_map)

# 拼接4个小地图为全局地图字符串
def merge_global_map(map_list, small_cols, small_rows):
    map1, map2, map3, map4 = map_list
    global_map = []
    # 上半部分：左上 + 右上
    for i in range(small_rows):
        row1 = map1[i*small_cols : (i+1)*small_cols]
        row2 = map2[i*small_cols : (i+1)*small_cols]
        global_map.append(row1 + row2)
    # 下半部分：左下 + 右下
    for i in range(small_rows):
        row3 = map3[i*small_cols : (i+1)*small_cols]
        row4 = map4[i*small_cols : (i+1)*small_cols]
        global_map.append(row3 + row4)
    return ''.join(global_map)

# 执行4ROI检测并返回全局地图
def detect_all_roi(grid_cols, grid_rows):
    map_results = []
    for idx, (start_x, start_y) in enumerate(ROI_COORDINATES):
        res = detect_single_roi(grid_cols, grid_rows, start_x, start_y)
        map_results.append(res)
        time.sleep(0.1)
    return merge_global_map(map_results, grid_cols, grid_rows)

# ==================== 主循环 ====================
if __name__ == "__main__":
    cols, rows = init_camera()
    while True:
        if uart.any():
            data = uart.read(1)
            # 收到 0x01 触发一次全地图识别
            if data and data[0] == 0x01:
                # 执行4ROI检测 + 地图拼接
                global_result = detect_all_roi(cols, rows)

                # 串口输出全局地图
                uart.write(f"{global_result}\r\n")

                # 蓝灯提示
                led_blue.on()
                time.sleep_ms(20)
                led_blue.off()
        time.sleep_ms(10)
