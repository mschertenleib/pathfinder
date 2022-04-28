import matplotlib.pyplot as plt
import numpy as np


infinity = 1_000_000


def generate_walls(height, width):
    #walls = np.random.rand(height, width) > 0.95
    walls = np.zeros((height, width), dtype=np.bool8)
    for y in range(10, height - 5):
        walls[y][width // 2] = True

    return walls


def is_cell_walkable(walls, cell_x: int, cell_y: int, radius: int):
    for x in range(-radius, radius + 1):
        y_max = int(np.sqrt(radius * radius - x * x))
        for y in range(-y_max, y_max + 1):
            if walls[cell_y + y][cell_x + x]:
                return False

    return True


def generate_walkable(walls, radius: int):
    height = walls.shape[0]
    width = walls.shape[1]
    walkable = np.zeros((height, width), dtype=np.bool8)
    for y in range(radius, height - radius):
        for x in range(radius, width - radius):
            walkable[y][x] = is_cell_walkable(walls, x, y, radius)

    return walkable
    

def print_image(walls, walkable, start, goal):
    height = walls.shape[0]
    width = walls.shape[1]
    image = np.zeros((height, width, 3), dtype=np.uint8)
    for y in range(height):
        for x in range(width):
            if walls[y][x]:
                image[y][x] = (0, 0, 0)
            elif walkable[y][x]:
                image[y][x] = (255, 255, 255)
            else:
                image[y][x] = (150, 150, 150)
    image[start] = (0, 200, 200)
    image[goal] = (200, 0, 0)

    plt.imshow(image)
    plt.show()


def main():
    width = 50
    height = 40
    robot_radius = 3
    start = (20, 10)
    goal = (height - 10, width - 10)
    walls = generate_walls(height, width)
    walkable = generate_walkable(walls=walls, radius=robot_radius)
    print_image(walls=walls, walkable=walkable, start=start, goal=goal)


if __name__ == '__main__':
    main()
