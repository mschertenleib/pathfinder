import numpy as np
import cv2
import environment_map as envmap
import EPuck2


def main():
    true_map = envmap.Environment_map(100, 100, 1)
    true_map.set_all_free()
    true_map.set_occupied_line_normalized((0.2, 0.3), (0.8, 0.1))
    true_map.set_occupied_line_normalized((0.5, 0.7), (0.8, 0.7))
    true_map.set_occupied_line_normalized((0.1, 0.1), (0.1, 0.9))
    true_map.set_occupied_line_normalized((0.7, 0.3), (0.7, 0.7))
    true_map.set_occupied_rectangle_normalized((0.38, 0.65), (0.4, 1))
    
    img = true_map.as_image().astype(float) / 255
    img = 1 - img
    
    line_to_add = np.zeros((100,100), dtype=np.uint8)
    cv2.line(img=line_to_add, pt1=(0,0), pt2=(80,80), color=255, thickness=1, lineType=cv2.LINE_AA)
    img = cv2.add(img, line_to_add.astype(float) / 255)
    
    img = cv2.resize(img, dsize=(800, 800), interpolation=cv2.INTER_NEAREST)
    
    cv2.imshow('image', img)
    
    cv2.waitKey()



if __name__ == '__main__':
    main()
