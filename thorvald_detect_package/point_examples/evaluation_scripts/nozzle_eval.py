import numpy as np
import pandas as pd


weed_x = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedx.csv',names=["weed_x"])
weed_y = pd.read_csv('/home/computing/paper_ws/src/Thorvald_weed_spraying/thorvald_detect_package/point_examples/points1/weedy.csv',names=["weed_y"])
weed_x = weed_x.values
weed_y = weed_y.values

# Sprayer characteristics
spray_radius = 0.5

y_spray = [-0.4,-0.2875,-0.175,-0.0625,0.05,0.1625,0.275,0.375,0.5]

dist_spray = []

# Speeds
v_spray = 10
v_robot = 0.5

# Counters
not_sprayed = 0
sprayer1 = 0
sprayer2 = 0
sprayer3 = 0
sprayer4 = 0
sprayer5 = 0
sprayer6 = 0
sprayer7 = 0
sprayer8 = 0
sprayer9 = 0
sprayer10 = 0

dist_spray =  y_spray-weed_y[0]

for i in range(0,len(weed_x)-1):
    # get points
    x_dist = weed_x[i+1] - weed_x[i]
    y_dist = weed_y[i+1] - weed_y[i]

    # find dist between point and each nozzle
    dist_sprayer = abs(y_spray - weed_y[i])
    
    # find min dist
    min_dist = np.min(dist_sprayer)

    # find nozzle closest to weed
    min_position = [i for i, x in enumerate(dist_sprayer) if x == min_dist]
    min_sprayer = min_position[0]

    if min_dist > 0.025:
       
        not_sprayed = not_sprayed + 1

    elif min_sprayer == 0:
       
        sprayer1 = sprayer1 + 1
       
    elif min_sprayer == 1:
       
        sprayer2 = sprayer2 + 1
       
    elif min_sprayer == 2:
       
        sprayer3 = sprayer3 + 1
    
    elif min_sprayer == 3:
       
        sprayer4 = sprayer4 + 1
       
    elif min_sprayer == 4:
       
        sprayer5 = sprayer5 + 1
    elif min_sprayer == 5:
       
        sprayer6 = sprayer6 + 1
       
    elif min_sprayer == 6:
       
        sprayer7 = sprayer7 + 1

    elif min_sprayer == 7:
       
        sprayer8 = sprayer8 + 1
    elif min_sprayer == 8:
       
        sprayer9 = sprayer9 + 1
    elif min_sprayer == 9:
       
        sprayer10 = sprayer10 + 1
       


print("total sprayer1 = ", sprayer1)
print("total sprayer2 = ", sprayer2)
print("total sprayer3 = ", sprayer3)

print("total sprayer4 = ", sprayer4)
print("total sprayer5 = ", sprayer5)

print("total sprayer6 = ", sprayer6)
print("total sprayer7 = ", sprayer7)
print("total sprayer8 = ", sprayer8)

print("total sprayer9 = ", sprayer9)
print("total sprayer10 = ", sprayer10)

print("total not sprayed = ", not_sprayed)