import matplotlib
import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter
import sys
import numpy as np
import math

robot = [5, 10, 15]
robot2 = [5, 10, 15, 20]

open_makespan_avg = [5.9, 6.1, 7.1]
open_soc_avg = [20.3, 38.8, 60.4]
open_soc_avg_per_robot = [4.06, 3.88, 4.026666667]

open_compute_time = [0.7314621925, 6.793066788, 43.00468268]

open_success_rate = [1, 1, 1, 0.2]

cluttered_makespan_avg = [7.2, 7.1, 8.4]
cluttered_soc_avg = [24, 43.4, 66]
cluttered_soc_avg_per_robot = [4.8, 4.34, 4.4]

cluttered_compute_time = [31.68323803, 74.12248969, 214.7697276]

cluttered_success_rate = [1, 1, 0.5, 0.1]

font_size1 = 25.5 * 1.2
font_size2 = 32 * 1.2
line_width1 = 10
line_width2 = 6
marker_size1 = 25
marker_size2 = 15

# draw graph by matplotlib
# draw graph for open_makespan_avg, open_soc_avg, open_soc_avg_per_robot in red line in one graph
# draw graph for cluttered_makespan_avg, cluttered_soc_avg, cluttered_soc_avg_per_robot in blue line in one graph

# plt.figure(figsize=(10, 10))
fig, ax = plt.subplots(figsize=(10, 10))
# plt.plot(robot, open_makespan_avg, '-o', color='red', markersize=marker_size1, linewidth=line_width1, label='Open')
#plt.plot(robot, open_soc_avg, '--o', color='red', label='open_soc_avg')
#plt.plot(robot, open_soc_avg_per_robot, ':o', color='red', label='open_soc_avg_per_robot')
# plt.plot(robot, cluttered_makespan_avg, '-o', color='blue', markersize=marker_size1, linewidth=line_width1, label='Cluttered')
#plt.plot(robot, cluttered_soc_avg, '--o', color='blue', label='cluttered_soc_avg')
#plt.plot(robot, cluttered_soc_avg_per_robot, ':o', color='blue', label='cluttered_soc_avg_per_robot')

ax.plot(robot, open_makespan_avg, '-o', markersize=marker_size1, linewidth=line_width1, label='Open')
ax.plot(robot, cluttered_makespan_avg, '-o', markersize=marker_size1, linewidth=line_width1, label='Cluttered')

ax.legend(loc='upper left', prop={'size': font_size1})  # 레전드 라벨의 글꼴 크기 설정
ax.set_xlabel('Number of agents', fontsize=font_size2)  # x축 레이블의 글꼴 크기 설정
ax.set_ylabel('Makespan', fontsize=font_size2)  # y축 레이블의 글꼴 크기 설정

# ax.xticks(robot, fontsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
# ax.yticks(fontsize=font_size2)  # y축 눈금 레이블의 글꼴 크기 설정

ax.set_xticklabels(robot, fontsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='x', labelsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='y',labelsize=font_size2)  # y축 눈금 레이블의 글꼴 크기 설정

#plt.yscale('log')  # y축을 로그 스케일로 설정
# ax.gca().yaxis.set_major_formatter(ScalarFormatter())  # 눈금 레이블 설정
# ax.gca().yaxis.set_minor_formatter(ScalarFormatter())  # 보조 눈금 레이블 설정
# ax.gca().yaxis.set_tick_params(which='both', labelsize=font_size2)  # y축 보조눈금 레이블의 글꼴 크기 설정

fig.subplots_adjust(right=0.8)
plt.tight_layout()

plt.show()

###################################################################################

fig, ax = plt.subplots(figsize=(10, 10))
#plt.plot(robot, open_makespan_avg, '-o', color='red', label='open_makespan_avg')
#plt.plot(robot, open_soc_avg, '--o', color='red', label='open_soc_avg')
# plt.plot(robot, open_soc_avg_per_robot, '-o', color='red', markersize=marker_size1, linewidth=line_width1, label='Open')
#plt.plot(robot, cluttered_makespan_avg, '-o', color='blue', label='cluttered_makespan_avg')
#plt.plot(robot, cluttered_soc_avg, '--o', color='blue', label='cluttered_soc_avg')
# plt.plot(robot, cluttered_soc_avg_per_robot, '-o', color='blue', markersize=marker_size1, linewidth=line_width1, label='Cluttered')

ax.plot(robot, open_soc_avg_per_robot, '-o', markersize=marker_size1, linewidth=line_width1, label='Open')
ax.plot(robot, cluttered_soc_avg_per_robot, '-o', markersize=marker_size1, linewidth=line_width1, label='Cluttered')

ax.legend(loc='upper right', prop={'size': font_size1})  # 레전드 라벨의 글꼴 크기 설정
ax.set_xlabel('Number of agents', fontsize=font_size2)  # x축 레이블의 글꼴 크기 설정
ax.set_ylabel('Sum of costs per agent', fontsize=font_size2)  # y축 레이블의 글꼴 크기 설정
ax.set_xticklabels(robot, fontsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='x', labelsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='y',labelsize=font_size2)  # y축 눈금 레이블의 글꼴 크기 설정

#plt.yscale('log')  # y축을 로그 스케일로 설정
# ax.gca().yaxis.set_major_formatter(ScalarFormatter())  # 눈금 레이블 설정
# ax.gca().yaxis.set_minor_formatter(ScalarFormatter())  # 보조 눈금 레이블 설정
# ax.gca().yaxis.set_tick_params(which='both', labelsize=font_size2)  # y축 보조눈금 레이블의 글꼴 크기 설정

fig.subplots_adjust(right=0.8)
plt.tight_layout()

plt.show()

###################################################################################

fig, ax = plt.subplots(figsize=(10, 10))
# plt.plot(robot, open_compute_time, '-o', color='red', markersize=marker_size1, linewidth=line_width1, label='Open')
# plt.plot(robot, cluttered_compute_time, '-o', color='blue', markersize=marker_size1, linewidth=line_width1, label='Cluttered')

ax.plot(robot, open_compute_time, '-o', markersize=marker_size1, linewidth=line_width1, label='Open')
ax.plot(robot, cluttered_compute_time, '-o', markersize=marker_size1, linewidth=line_width1, label='Cluttered')

ax.legend(loc='upper left', prop={'size': font_size1})  # 레전드 라벨의 글꼴 크기 설정
ax.set_xlabel('Number of agents', fontsize=font_size2)  # x축 레이블의 글꼴 크기 설정
ax.set_ylabel('Computation time (s)', fontsize=font_size2)  # y축 레이블의 글꼴 크기 설정
ax.set_xticklabels(robot, fontsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='x', labelsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정
ax.tick_params(axis='y',labelsize=font_size2)  # y축 눈금 레이블의 글꼴 크기 설정

fig.subplots_adjust(right=0.8)
plt.tight_layout()

plt.show()

###################################################################################

bar_width = 0.3
bar_positions1 = np.arange(len(robot2))
bar_positions2 = [x + bar_width + 0.1 for x in bar_positions1]

fig, ax = plt.subplots(figsize=(10, 10))

# Plot the first dataset (Open)
# plt.bar(bar_positions1, open_success_rate, width=bar_width, color='red', label='Open')
ax.bar(bar_positions1, open_success_rate, width=bar_width, label='Open')

# Plot the second dataset (Cluttered)
# plt.bar(bar_positions2, cluttered_success_rate, width=bar_width, color='blue', label='Cluttered')
ax.bar(bar_positions2, cluttered_success_rate, width=bar_width, label='Cluttered')

ax.legend(loc='upper right', prop={'size': font_size1})  # 레전드 라벨의 글꼴 크기 설정
ax.set_xlabel('Number of agents', fontsize=font_size2)  # x축 레이블의 글꼴 크기 설정
ax.set_ylabel('Success rate', fontsize=font_size2)  # y축 레이블의 글꼴 크기 설정

# Adjust the positions and labels of x-ticks
middle_x = [(a + b) / 2 for (a, b) in zip(bar_positions1, bar_positions2)]
ax.set_xticks(middle_x)  # x축 눈금 레이블의 위치 설정
ax.set_xticklabels(robot2, fontsize=font_size2)  # x축 눈금 레이블의 글꼴 크기 설정

ax.tick_params(axis='y', labelsize=font_size2)  # y축 눈금 레이블의 글꼴 크기 설정

ax.set_xlim(bar_positions1[0] - 0.2, bar_positions2[-1] + bar_width - 0.1)  # X 축 범위 설정

fig.subplots_adjust(right=0.8)
plt.tight_layout()

plt.show()
