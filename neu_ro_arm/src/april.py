import numpy as np
import matplotlib.pyplot as plt

from src.camera import Camera
import cv2

n_cols = 4
n_rows = 3

markers = []
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
side_pixels = 120
border_bits = 0
white_padding_size = 20
black_padding_size = 20
id_ = 0

texts = []
text_orgs = []
for c_id in range(n_cols):
    markers.append([])
    for r_id in range(n_rows):
        if id_ == 13:
            id_ += 1
        id_ = c_id + r_id*n_cols
        marker = cv2.aruco.drawMarker(aruco_dict, id_, side_pixels, border_bits)
        marker = np.pad(marker, white_padding_size, mode='constant', constant_values=255)
        marker = np.pad(marker, black_padding_size, mode='constant', constant_values=0)
        markers[c_id].append(marker)
        texts.append(f"tag{id_}")
        text_orgs.append((int((c_id+0.43)*marker.shape[0]),
                          int((r_id+0.08)*marker.shape[1])) )
        id_ += 1
tile_shape = markers[0][0].shape

# tile the image
markers = np.concatenate(markers, axis=-1)
markers = np.concatenate(markers, axis=0)

# add tag ids
for txt, org in zip(texts, text_orgs):
    cv2.putText(markers, txt, org, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.5, color=(255,255,255), thickness=1, lineType=cv2.LINE_AA)


cv2.imshow(';sadf',markers)
cv2.imwrite('data/aruco_markers.png', markers)
cv2.waitKey(5000)

# img = cv2.imread('data/apriltag-mosiac-36h11.png')

# # img = cv2.copyMakeBorder(img, 1,0,1,0,cv2.BORDER_CONSTANT, value=0)
# print(img.shape)
# num_tags = 12
# size = 10
# tags = []

# out = np.zeros((3*(size+1)+1, 4*(size+1)+1, 3), dtype=np.uint8)
# for i in range(num_tags):
    # x,y = i%3, i//3
    # out[1+x*(size+1):1+(x+1)*size+x, 1+y*(size+1):1+(y+1)*size+y] = img[0:size,i*(size+1):i*(size+1)+size]
# h,w = out.shape[:2]
# dilation = 20
# H = dilation*h
# W = dilation*w
# out = cv2.resize(out, (W,H), interpolation=cv2.INTER_NEAREST_EXACT)

# name = 'april tags'
# cv2.namedWindow(name)
# for i in range(num_tags):
    # y,x = i%3, i//3
    # org = ( int(dilation*(x+0.44)*(size+1)),int(dilation*(y+0.98)*(size+1)))
    # out = cv2.putText(out, f'tag{i}', org, fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                # fontScale=0.5, color=(0,0,0), thickness=1, lineType=cv2.LINE_AA)
# cv2.imshow(name, out)
# cv2.waitKey(6000)
# cv2.imwrite('data/apriltag-mosiac-36h11-bigger.png', out)
