import pygame
import sys

# Initialize Pygame
pygame.init()

# Constants
WINDOW_SIZE = (800, 800)
DOT_RADIUS = 10
MAZE_FILE = "Shifter.png"  # Put your black and white maze image here

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

# Initialize screen and clock
screen = pygame.display.set_mode(WINDOW_SIZE)
pygame.display.set_caption("Maze Game")
clock = pygame.time.Clock()

# Load maze
maze = pygame.image.load(MAZE_FILE)
maze = pygame.transform.scale(maze, WINDOW_SIZE)


def closest_boundary_point(x, y):
    dx, dy = 0, 0
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Right, Down, Left, Up
    length = 0
    direction_index = 0

    while True:
        for i in range(2):
            length += 1
            for _ in range(length):
                try:
                    if maze.get_at((x + dx, y + dy)) != BLACK:
                        return x + dx, y + dy
                except IndexError:
                    pass

                dx += directions[direction_index][0]
                dy += directions[direction_index][1]

            direction_index = (direction_index + 1) % 4


def check_collision(x, y, radius):
    collision = False
    for dx in range(-radius, radius + 1):
        for dy in range(-radius, radius + 1):
            if dx * dx + dy * dy <= radius * radius:
                try:
                    if maze.get_at((x + dx, y + dy)) == BLACK:
                        collision = True
                        break
                except IndexError:
                    collision = True
                    break
    if collision:
        return True, closest_boundary_point(x, y)
    else:
        return False, (x, y)


def main():
    x, y = 50, 50  # Initial position of the dot

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()

        # Get joystick position (Replace these lines with your joystick reading code)
        joystick_x, joystick_y = pygame.mouse.get_pos()

        # Check for collision with maze boundaries
        collision, new_pos = check_collision(joystick_x, joystick_y, DOT_RADIUS)
        if collision:
            x, y = new_pos
        else:
            x, y = joystick_x, joystick_y

        # Draw everything
        screen.blit(maze, (0, 0))
        pygame.draw.circle(screen, RED, (x, y), DOT_RADIUS)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    main()
