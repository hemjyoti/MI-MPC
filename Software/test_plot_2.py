import numpy as np
import pygame_plot
import pygame
pygame.init
SIZE = (13,6)
screen = pygame.display.set_mode(SIZE)
BLACK = (0,0,0)
WHITE = (255,255,255)
BLUE = (0,0,255)
RED = (0,0,0)
THICKNESS = 3

screen.fill(BLACK)
WHITE = (255,255,255)
kybrd104 = [(1,2),(3,4),(5,4),(11,4),(11,3.5)]

pygame.draw.lines(screen, WHITE, False, kybrd104, THICKNESS)
pygame.display.update()

while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit(),
            sys.exit()
        print(event)
        pygame.display.update()
        break