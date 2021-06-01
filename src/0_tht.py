#used code
#https://stackoverflow.com/questions/18094928/in-matplotlib-what-is-the-polar-equivalent-of-get-xdata
#https://stackoverflow.com/questions/7969352/matplotlib-interactively-select-points-or-locations    
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import pickle as pkl

# first select the left one hole for an element, then the right one.
# after you selected all needed points, close Matplotlib window, 
# and points will be saved in the place_positions.pkl


df = pd.read_csv("drill.csv")
df.head()

fig = plt.figure()
ax = fig.add_subplot(111)
ax.set_title('click on points')
line, = ax.plot(df["x"],df["y"], 'x', picker=5)  # 5 points tolerance
flag_first_second=False
points=[]
last_element=[(0,0),(0,0)]
def onpick(event):
    global flag_first_second
    global points
    global last_element
    thisline = event.artist
    xdata = thisline.get_xdata()
    ydata = thisline.get_ydata()
    ind = event.ind
    ax.scatter(xdata[ind], ydata[ind], color = "red")
    plt.show()

    
    selected_point = (xdata[ind][0], ydata[ind][0])
    if flag_first_second==False:
        last_element[0]=selected_point
        flag_first_second=True
        print('onpick points:',selected_point )
    else:
        flag_first_second=False
        last_element[1]=selected_point
        points.append(last_element)
        last_element=[(0,0),(0,0)]
        print('onpick points:',selected_point, "points", points)

fig.canvas.mpl_connect('pick_event', onpick)

plt.show()
with open("place_positions.pkl", 'wb') as handle:
    pkl.dump(points, handle, protocol=4)
    print("Points saved in: ","place_positions.pkl")
