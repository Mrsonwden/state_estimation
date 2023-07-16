import pandas as pd
import matplotlib.pyplot as plt
root_path = "/home/snowden/workplace/state_estimation_for_robotics/state_estimation"
csv_file_path = f"{root_path}/data/kf_result.csv"

data = pd.read_csv(csv_file_path)

# 绘制每列的数据
plt.figure(figsize=(60,60))

for column in data.columns:
    plt.plot(data[column], label=column, linewidth=0.2)

# 添加图例
plt.legend()

# 显示图像
# plt.show()
plt.savefig( f"{root_path}/build/my_figure.png")
