import numpy as np
import pandas as pd


x = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedx.csv',names=["weed_x"])
y = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedy.csv',names=["weed_y"])
weed_x = x.values
weed_y = y.values

x_dist =[]
y_dist = []
spray = []
i = 0

v_spray = 4
v_robot = 0.5

not_sprayed = 0
sprayed = 0

while i < len(weed_x)-1 :

    x_dist = weed_x[i+1] - weed_x[i]
    y_dist = weed_y[i+1] - weed_y[i]

    y_delay =  abs(y_dist)/v_spray
    x_delay =  abs(x_dist)/v_robot 

    if y_delay > x_delay:
        spray.append(0)
        not_sprayed = not_sprayed + 1
       
    else:
        spray.append(1)
        sprayed = sprayed + 1

    i = i+1

print("total sprayed = ", sprayed)
print("total not sprayed = ", not_sprayed)

spray = pd.DataFrame(data=spray,columns=["sprayed"])

complete = pd.concat([x,y,spray], axis=1)
print(complete.head())
# complete.to_csv("/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/evaluation_scripts/complete_single_1.csv", index=False)