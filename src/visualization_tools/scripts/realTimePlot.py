#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rospy
from std_msgs.msg import Float32

mpl.rcParams['toolbar'] = 'None'
plt.ion()

# 异步从 ROS 接收的最新值。绘图循环采样这些快照，而不是在订阅回调中直接绘制。
time_duration = 0
start_time_duration = 0
first_iteration = 'True'

explored_volume = 0;
traveling_distance = 0;
run_time = 0;
max_explored_volume = 0
max_traveling_diatance = 0
max_run_time = 0

time_list1 = np.array([])
time_list2 = np.array([])
time_list3 = np.array([])
run_time_list = np.array([])
explored_volume_list = np.array([])
traveling_distance_list = np.array([])

def timeDurationCallback(msg):
    # 用第一条指标消息而非墙上时间作为绘图时间原点。
    global time_duration, start_time_duration, first_iteration
    time_duration = msg.data
    if first_iteration == 'True':
        start_time_duration = time_duration
        first_iteration = 'False'

def runTimeCallback(msg):
    # /runtime 由可选的外部算法性能分析器提供。
    global run_time
    run_time = msg.data

def exploredVolumeCallback(msg):
    global explored_volume
    explored_volume = msg.data


def travelingDistanceCallback(msg):
    global traveling_distance
    traveling_distance = msg.data

def listener():
  global time_duration, start_time_duration, explored_volume, traveling_distance, run_time, max_explored_volume, max_traveling_diatance, max_run_time, time_list1, time_list2, time_list3, run_time_list, explored_volume_list, traveling_distance_list

  rospy.init_node('realTimePlot')
  rospy.Subscriber("/time_duration", Float32, timeDurationCallback)
  rospy.Subscriber("/runtime", Float32, runTimeCallback)
  rospy.Subscriber("/explored_volume", Float32, exploredVolumeCallback)
  rospy.Subscriber("/traveling_distance", Float32, travelingDistanceCallback)

    # 保留三份历史序列，因为每个指标可能使用独立坐标轴。
  fig=plt.figure(figsize=(8,7))
  fig1=fig.add_subplot(311)
  plt.title("Exploration Metrics\n", fontsize=14)
  plt.margins(x=0.001)
  fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
  l1, = fig1.plot(time_list2, explored_volume_list, color='r', label='Explored Volume')
  fig2=fig.add_subplot(312)
  fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
  l2, = fig2.plot(time_list3, traveling_distance_list, color='r', label='Traveling Distance')
  fig3=fig.add_subplot(313)
  fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
  fig3.set_xlabel("Time Duration (s)", fontsize=12) # 仅需设置一次
  l3, = fig3.plot(time_list1, run_time_list, color='r', label='Algorithm Runtime')

  count = 0
  r = rospy.Rate(100) # 以 100 Hz 驱动输入采样和 GUI 刷新调度。
  while not rospy.is_shutdown():
      r.sleep()
      count = count + 1

      if count % 25 == 0:
        # 以 4 Hz 追加指标快照；两次采样之间 ROS 回调仍可按自身频率更新最新值。
        max_explored_volume = explored_volume
        max_traveling_diatance = traveling_distance
        if run_time > max_run_time:
            max_run_time = run_time

        time_list2 = np.append(time_list2, time_duration)
        explored_volume_list = np.append(explored_volume_list, explored_volume)
        time_list3 = np.append(time_list3, time_duration)
        traveling_distance_list = np.append(traveling_distance_list, traveling_distance)
        time_list1 = np.append(time_list1, time_duration)
        run_time_list = np.append(run_time_list, run_time)

      if count >= 100:
        # 每秒仅重绘一次，避免 GUI 成为性能瓶颈。
        count = 0
        l1.set_xdata(time_list2)
        l2.set_xdata(time_list3)
        l3.set_xdata(time_list1)
        l1.set_ydata(explored_volume_list)
        l2.set_ydata(traveling_distance_list)
        l3.set_ydata(run_time_list)

        fig1.set_ylim(0, max_explored_volume + 500)
        fig1.set_xlim(start_time_duration, time_duration + 10)
        fig2.set_ylim(0, max_traveling_diatance + 20)
        fig2.set_xlim(start_time_duration, time_duration + 10)
        fig3.set_ylim(0, max_run_time + 0.2)
        fig3.set_xlim(start_time_duration, time_duration + 10)

        fig.canvas.draw()

if __name__ == '__main__':
  listener()
  print("1")
