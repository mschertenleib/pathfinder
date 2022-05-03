import numpy as np
import cv2
from pathfinding.core.diagonal_movement import DiagonalMovement
from pathfinding.core.grid import Grid
from pathfinding.finder.a_star import AStarFinder


def main():
    height = 48
    width = 64
    matrix = np.random.rand(height, width) > 0.4
    start_coords = (5, 5)
    goal_coords = (height - 5, width - 5)
    matrix[start_coords] = True
    matrix[goal_coords] = True

    grid = Grid(matrix=matrix)
    start = grid.node(start_coords[1], start_coords[0])
    end = grid.node(goal_coords[1], goal_coords[0])

    finder = AStarFinder(diagonal_movement=DiagonalMovement.always)
    print('Pathfinding started...')
    path, runs = finder.find_path(start, end, grid)
    print('Pathfinding finished')

    print('Operations:', runs)
    if len(path) > 0:
        print('Path length:', len(path))
    else:
        print('No path from start to goal found')

    img = np.zeros((height, width, 3))
    for i in range(height):
        for j in range(width):
            if matrix[i][j]:
                img[i][j] = (255, 255, 255)
    for x, y in path:
        img[y][x] = (255, 255, 0)
    img[start_coords] = (0, 255, 0)
    img[goal_coords] = (0, 0, 255)
    img = cv2.resize(src=img, dsize=(640, 480),
                     interpolation=cv2.INTER_NEAREST)
    cv2.imshow('map', img)
    cv2.waitKey()


if __name__ == '__main__':
    main()
