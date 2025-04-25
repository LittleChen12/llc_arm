import serial
import threading
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

# 配置 Matplotlib 使用支持中文的字体
plt.rcParams["font.sans-serif"] = ["SimHei"]
plt.rcParams["axes.unicode_minus"] = False

# -----------------------------
# 全局数据（均保留，不进行截断）
# -----------------------------
x_data = []        # 存储 x 坐标
y_data = []        # 存储 y 坐标
adc_data = []      # 存储 ADC 数值（单位 mV）
defect_flags = []  # 存储每个数据点是否存在缺陷（True=缺陷，False=正常）

# 原来的 ADC 差分检测不再使用，可删除 last_adc
# last_adc = None
DEFECT_THRESHOLD = 20.0

# 为了优化动画性能，即使全量数据保留，我们在显示时只显示部分点
DISPLAY_MAX_POINTS = 2000  # 显示上限，超过时自动下采样

# 正则表达式，匹配 "x:数字,y:数字,adc:数字" 格式的数据
pattern = re.compile(r"x:([-+]?[0-9]*\.?[0-9]+),y:([-+]?[0-9]*\.?[0-9]+),adc:([-+]?[0-9]*\.?[0-9]+)")

# 定义 y 轴数据的滑动窗口，大小为 20
WINDOW_SIZE = 20
adc_window = deque(maxlen=WINDOW_SIZE)


def read_serial_data(ser):
    """
    持续从串口读取数据，解析出 x, y, adc 数值。
    · 维护一个 ADC 数值的滑动窗口（大小20）。
    · 计算窗口内最小值，与所有数据做差值分析。
    · 若最大差值超过 DEFECT_THRESHOLD，则标记为缺陷。
    """
    global x_data, y_data, adc_data, defect_flags, adc_window
    while True:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            if line:
                matches = pattern.findall(line)
                if not matches:
                    print("数据格式错误:", line)
                    continue
                for m in matches:
                    try:
                        x_val = float(m[0])
                        y_val = float(m[1])
                        adc_val = float(m[2])
                    except ValueError:
                        print("数据转换错误:", m)
                        continue

                    # 将 ADC 数据添加到滑动窗口
                    adc_window.append(adc_val)

                    # 默认认为无缺陷
                    defect = False

                    if len(adc_window) == WINDOW_SIZE:
                        # 计算窗口内最小 ADC 值
                        min_adc = min(adc_window)
                        # 计算窗口内数据与最小值的绝对差
                        diffs = [abs(val - min_adc) for val in adc_window]
                        max_diff = max(diffs)

                        if max_diff > DEFECT_THRESHOLD:
                            defect = True

                    x_data.append(x_val)
                    y_data.append(y_val)
                    adc_data.append(adc_val)
                    defect_flags.append(defect)

            # 调试输出
            print("收到数据:", line)
            print("滑动窗口数据 (ADC):", list(adc_window))
        except Exception as e:
            print("串口读取错误：", e)
            break


# -----------------------------
# 绘图对象与动画函数
# -----------------------------
# 散点图（上图）用于展示扫描平面数据：蓝色表示正常点、红色表示缺陷点
scat_normal = None
scat_defect = None
# ADC 折线图（下图）用于展示实时 ADC 数值
line_adc = None

# 全局子图句柄（在 main() 中初始化）
ax_scatter = None  # 上图：散点图
ax_adc = None      # 下图：ADC 折线图


def init():
    """
    动画初始化：
    · 上图中预建两个散点对象（正常和缺陷）；
    · 下图中预建 ADC 折线图对象。
    """
    global scat_normal, scat_defect, line_adc, ax_scatter, ax_adc
    scat_normal = ax_scatter.scatter([], [], c='blue', s=2000, alpha=0.5)
    scat_defect = ax_scatter.scatter([], [], c='red', s=2000, alpha=1.0)
    ax_scatter.set_xlabel("X 坐标")
    ax_scatter.set_ylabel("Y 坐标")
    ax_scatter.set_title("扫描平面缺陷展示：红色=缺陷, 蓝色=正常")
    ax_scatter.grid(True)

    # 下图：ADC 数据折线图，横轴为数据采样序号
    line_adc, = ax_adc.plot([], [], color='green', lw=2)
    ax_adc.set_xlabel("样本索引")
    ax_adc.set_ylabel("ADC 数值")
    ax_adc.set_title("实时 ADC 数值")
    ax_adc.grid(True)

    return scat_normal, scat_defect, line_adc,


def animate(frame):
    global scat_normal, scat_defect, line_adc, ax_scatter, ax_adc
    # 复制全局数据，以确保数据一致性
    x_copy = x_data[:]
    y_copy = y_data[:]
    adc_copy = adc_data[:]
    flag_copy = defect_flags[:]

    if not x_copy or not y_copy:
        return scat_normal, scat_defect, line_adc,

    # 使用复制的数据计算数据点数
    N = len(x_copy)
    if N > DISPLAY_MAX_POINTS:
        step = N // DISPLAY_MAX_POINTS
        display_indices = np.arange(0, N, step)
    else:
        display_indices = np.arange(N)

    display_x = np.array(x_copy)[display_indices]
    display_y = np.array(y_copy)[display_indices]
    display_flags = np.array(flag_copy)[display_indices]

    # 这里约定：在散点图中，以 (display_y, display_x) 作为显示顺序
    coords = np.column_stack((display_y, display_x))
    normal_indices = [i for i, flag in enumerate(display_flags) if not flag]
    defect_indices = [i for i, flag in enumerate(display_flags) if flag]

    normal_coords = coords[normal_indices] if normal_indices else np.empty((0, 2))
    defect_coords = coords[defect_indices] if defect_indices else np.empty((0, 2))

    scat_normal.set_offsets(normal_coords)
    scat_defect.set_offsets(defect_coords)

    # 调整坐标轴范围
    ax_scatter.set_xlim(min(y_copy) - 0.01, max(y_copy) + 0.01)
    ax_scatter.set_ylim(min(x_copy) - 0.01, max(x_copy) + 0.01)

    # 使用 adc_copy 数据更新 ADC 折线图
    indices = np.arange(len(adc_copy))
    line_adc.set_data(indices, adc_copy)
    ax_adc.set_xlim(0, len(adc_copy) + 1)
    if adc_copy:
        ax_adc.set_ylim(min(adc_copy) - 1, max(adc_copy) + 1)

    return scat_normal, scat_defect, line_adc,



def main():
    global ax_scatter, ax_adc
    com_port = "COM8"  # 修改为实际串口号
    baud_rate = 1500000  # 修改为实际波特率
    try:
        ser = serial.Serial(com_port, baud_rate, timeout=1)
        print(f"成功打开串口：{com_port}")
    except Exception as e:
        print("打开串口失败：", e)
        return

    # 启动后台线程，持续读取串口数据（所有数据全部保留，不截断）
    thread = threading.Thread(target=read_serial_data, args=(ser,))
    thread.daemon = True
    thread.start()

    # 创建包含两个子图的窗口：上图为散点图，下图为 ADC 折线图
    fig, (ax_scatter, ax_adc) = plt.subplots(2, 1, figsize=(10, 8))
    fig.suptitle("实时扫描数据展示")
    fig.tight_layout(rect=[0, 0, 1, 0.96])

    # 创建动画对象，定时调用 animate() 更新显示
    ani = animation.FuncAnimation(fig, animate, init_func=init,
                                  interval=10, cache_frame_data=False)
    plt.show()

    ser.close()
    print("串口已关闭")


if __name__ == "__main__":
    main()
