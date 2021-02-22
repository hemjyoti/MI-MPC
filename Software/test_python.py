#file = open("helix_traj.txt")
#for line in file:
    #print(line)
#    field = line.split(" ")
#    field1 = field[1]
#    print(field1)
#file.close()

#import matplotlib.pyplot as plt
#from matplotlib import animation
#x=[]
#y=[]
#count_x=0
#count_y=2

#def draw_graph(i):
#    global count_y
#    global count_x
#    count_x = count_x + 1
#    count_y = count_y + 2*count_y
#    x.append(count_x)
#    y.append(count_y)

#    plt.cla()
#    plt.plot(x,y)

#for i in range(2,9):
#    anima = animation.FuncAnimation(plt.gcf(),draw_graph,interval=500)
#    plt.show()

import numpy as np



#import matplotlib.pyplot as plt

#plt.axis([0, 10, 0, 1])
#x=[]
#y=[]
#count_x = 0
#count_y = 2
#for i in range(30):
#    count_x = count_x + 1
#    count_y = count_y + 2*count_y
#    x.append(count_x)
#    y.append(count_y)
#    plt.cla()
#    plt.plot(x, y)
#    plt.pause(0.001)
#plt.show()

# Tutorial for 3D plotting
#import matplotlib.pyplot as plt
#import numpy as np
#fig = plt.figure()
#ax = plt.axes(projection="3d")

#z_line = np.linspace(0, 15, 1000)
#x_line = np.cos(z_line)
#y_line = np.sin(z_line)
#ax.plot3D(x_line, y_line, z_line, 'red')

#z_line1 = np.linspace(0, 15, 1000)
#x_line1 = np.cos(2*z_line)
#y_line1 = np.sin(2*z_line)
#ax.plot3D(x_line1, y_line1, z_line1, 'green')

#plt.show()


#import matplotlib.pyplot as plt
#import pygame

#plt.axis([-1, 1, 0, 100])
#pygame.init()
#done = False
#clock = pygame.time.Clock()
#pygame.joystick.init()
#joystick_x = []
#y = []
#inc = 0.1
# -------- Main Program Loop -----------
#for i in range(30):
    # EVENT PROCESSING STEP
#    for event in pygame.event.get():
#        uy = 4

#    inc = inc + 1
#    joystick = pygame.joystick.Joystick(0)
#    joystick.init()

#    axis_test = joystick.get_axis(3)
#    joystick_x.append(axis_test)
#    y.append(inc)
#    axis_test_int = int(axis_test)
#    print(type(axis_test))
#    #isinstance(axis_test,int)
#    plt.cla()
#    plt.plot(y, y)
#    plt.pause(1)
#    #print(axis_test)
#    #print(' \n ')

#plt.show()
#pygame.quit()




import matplotlib.pyplot as plt
import pygame
from processing_py import *
import numpy as np
arr = np.array([70, 70])
app = App(600, 400) # create window: width, height
pygame.init()
done = False
clock = pygame.time.Clock()
pygame.joystick.init()
plt.axis([0, 10, 0, 1])

x = []
y = []
count_x = 0
count_y = 2
inc = 0.2
count_x_prev=0
count_y_prev=0
for i in range(1000000):

    for event in pygame.event.get():
        uy = 4

    inc = inc + 1
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    axis_test = joystick.get_axis(3)
    count_x = axis_test*1000
    count_y = axis_test*1000
    #x.append(count_x)
    #y.append(count_y)
    print(axis_test)
    app.background(0, 0, 0)  # set background:  red, green, blue
    #app.fill(255, 255, 0)  # set color for objects: red, green, blue
    #app.ellipse(app.mouseX, app.mouseY, arr[0], arr[1])  # draw a circle: center_x, center_y, size_x, size_y
    app.line(count_x_prev,count_y_prev, count_x, count_y)
    app.stroke(126)
    app.redraw()  # refresh the window
    count_x_prev=count_x
    count_y_prev=count_y
    #plt.cla()
    #plt.plot(x, y)
    #plt.pause(0.001)
#plt.show()
pygame.quit()