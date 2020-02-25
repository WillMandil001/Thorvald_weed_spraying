import numpy as np
import pandas as pd


x = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedx.csv',names=["weed_x"])
y = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedy.csv',names=["weed_y"])
weed_x = x.values
weed_y = y.values

