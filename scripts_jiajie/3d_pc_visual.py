import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def read_point_cloud(filename):
    """
    读取点云数据文件
    返回numpy数组形式的点云数据
    """
    points = []
    try:
        with open(filename, 'r') as f:
            for line in f:
                # 过滤空行
                if line.strip():
                    try:
                        # 分割每行的坐标值并转换为浮点数
                        x, y, z = map(float, line.strip().split(','))
                        points.append([x, y, z])
                    except ValueError as e:
                        print(f"警告: 跳过无效行: {line.strip()}")
                        continue
    except FileNotFoundError:
        print(f"错误: 找不到文件 {filename}")
        return None
    except Exception as e:
        print(f"错误: 读取文件时发生错误: {str(e)}")
        return None
    
    if not points:
        print("错误: 没有读取到有效的点云数据")
        return None
        
    return np.array(points)

def visualize_point_cloud(points):
    """
    可视化点云数据
    参数points: numpy数组形式的点云数据
    """
    if points is None or len(points) == 0:
        print("错误: 没有可视化的数据")
        return

    # 创建新的图形窗口
    plt.figure(figsize=(12, 8))
    ax = plt.axes(projection='3d')
    
    # 绘制散点图
    scatter = ax.scatter(points[:, 0],  # X坐标
                        points[:, 1],  # Y坐标
                        points[:, 2],  # Z坐标
                        c=points[:, 2],  # 用Z值着色
                        cmap='viridis',  # 使用viridis颜色映射
                        marker='o',
                        s=50,  # 点的大小
                        alpha=0.6)  # 透明度
    
    # 添加颜色条
    plt.colorbar(scatter, label='Z坐标值', shrink=0.8)
    
    # 设置轴标签
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    
    # 设置标题
    plt.title('3D点云可视化')
    
    # 设置初始视角
    ax.view_init(elev=20, azim=45)
    
    # 添加网格
    ax.grid(True)
    
    # 使坐标轴比例相等
    ax.set_box_aspect([1,1,1])
    
    # 打印统计信息
    print("\n点云统计信息:")
    print(f"总点数: {len(points)}")
    print("\n坐标范围:")
    print(f"X: {points[:, 0].min():.3f} 到 {points[:, 0].max():.3f}")
    print(f"Y: {points[:, 1].min():.3f} 到 {points[:, 1].max():.3f}")
    print(f"Z: {points[:, 2].min():.3f} 到 {points[:, 2].max():.3f}")
    
    # 显示图形
    plt.show()

def main():
    """
    主函数
    """
    # 文件名
    filename = '/home/jay/AGLoc_ws/map/picking_list.txt'
    
    # 读取点云数据
    print(f"正在读取文件: {filename}")
    points = read_point_cloud(filename)
    
    if points is not None:
        print(f"成功读取 {len(points)} 个点")
        # 可视化点云
        visualize_point_cloud(points)
    else:
        print("程序终止: 无法读取点云数据")

if __name__ == "__main__":
    main()