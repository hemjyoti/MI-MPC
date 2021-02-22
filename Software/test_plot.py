import matplotlib

import matplotlib.backends.backend_agg as agg

import pygame
from pygame.locals import *
import pylab

matplotlib.use("Agg")


fig = pylab.figure(figsize=[5,5], # Inches
                   dpi=100,        # 100 dots per inch, so the resulting buffer is 400x400 pixels
                   )
ax = fig.gca()


x = []
y = []
count_x = 0.1
count_y = 0.2
inc = 0.2
canvas = agg.FigureCanvasAgg(fig)
canvas.draw()
renderer = canvas.get_renderer()
raw_data = renderer.tostring_rgb()
pygame.init()
#pygame.joystick.init()

window = pygame.display.set_mode((1000, 800), DOUBLEBUF)
screen = pygame.display.get_surface()

size = canvas.get_width_height()

surf = pygame.image.fromstring(raw_data, size, "RGB")
screen.blit(surf, (0,0))
pygame.display.flip()

for i in range(2000000):

    for event in pygame.event.get():
        uy = 4

    inc = inc + 1
    #joystick = pygame.joystick.Joystick(0)
    #joystick.init()

    #axis_test = joystick.get_axis(3)
    count_x = count_x + 0.1
    count_y = count_y + .02*count_y
    x.append(count_x)
    y.append(count_y)
    #print(axis_test)
    ax.plot(x, y)
    #ax.pause(0.001)

ax.show()

#pygame.quit()