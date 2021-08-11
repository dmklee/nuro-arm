# This script is based on the article by Adrian Rosebrock at PyImageSearch
# https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/

import argparse
import os
import cv2
import numpy as np
from PIL import Image

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

PAPER_SHAPE = (215, 279) # mm
PAPER_MARGIN = 13 # mm
MARKER_PADDING = 10 # mm
MM2IN = 1.0 / 25.4
RESOLUTION = 200

def generate_tag(aruco_dict, tag_id, tag_size):
    '''Generate image of aruco tag as 2d array

    Parameters
    ----------
    aruco_dict : str
        name of aruco dictionary, from ARUCO_DICT
    tag_id : int
        integer identity of tag
    tag_size : float
        size of the tag when printed, proportional to size of output array;
        does not include the border pixels

    Returns
    -------
    ndarray
        2D array of dtype=np.uint8
    '''
    n_pixels = int(np.round(tag_size * MM2IN * RESOLUTION))
    tag = np.zeros((n_pixels, n_pixels), dtype=np.uint8)
    tag = cv2.aruco.drawMarker(aruco_dict, tag_id, n_pixels, tag, borderBits=1)
    return tag

def generate_tags(aruco_dict,
                  tag_ids,
                  tag_size,
                  destination):
    '''Generate pdf's that contain aruco tags

    Parameters
    ----------
    aruco_dict : str
        name of aruco dictionary, from ARUCO_DICT
    tag_ids : array_like of int
        integer identities of tags to be generated
    tag_size : float
        size of aruco tags in millimeters (this size does not include the border)
    destination : str
        folder name where the pdfs will be saved
    '''
    tags = []
    for tag_id in tag_ids:
        tag = generate_tag(aruco_dict, tag_id, tag_size)
        pad_width = int(np.round(MARKER_PADDING * MM2IN * RESOLUTION))
        padded_tag = np.pad(tag, ((pad_width, 0), (0, pad_width)),
                            constant_values=255 )
        tags.append(padded_tag)

    padded_tag_size = tag_size + 2 * MARKER_PADDING
    n_cols = int( (PAPER_SHAPE[0] - 2 * PAPER_MARGIN ) / padded_tag_size )
    n_rows = int( (PAPER_SHAPE[1] - 2 * PAPER_MARGIN ) / padded_tag_size )

    n_per_page = n_cols * n_rows
    for i in range(0, len(tags), n_per_page):
        j = min(i+n_per_page, len(tags))
        page_tags = [tags[i] for i in range(i,j)]

        n_dummy = (n_cols - (len(page_tags) % n_cols)) % n_cols
        [page_tags.append(255+np.zeros_like(tags[0])) for _ in range(n_dummy)]

        page = np.array(page_tags).reshape(-1, n_cols, *tags[0].shape)
        page = np.concatenate(page, axis=1)
        page = np.concatenate(page, axis=1)

        im = Image.fromarray(page)
        im.save(os.path.join(destination, f"tags{i}-{j}.pdf"),
                resolution=RESOLUTION)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generate printable Aruco markers')
    parser.add_argument('--size', '-s', type=float,
                        default=25,
                        help="size of the tag in mm")
    parser.add_argument('--number', '-n', type=int,
                        default=12,
                        help="number of tags to generate")
    parser.add_argument('--start-id', type=int,
                        default=0,
                        help="tag number to begin with")
    parser.add_argument('--destination', type=str,
                        required=True,
                        help="folder in which to save the pdfs")
    parser.add_argument('--dict-name', type=str,
                        default="DICT_4X4_50",
                        help="aruco tag dictionary name")
    args = parser.parse_args()

    assert args.dict_name in ARUCO_DICT
    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[args.dict_name])

    assert 10 < args.size < 150, \
            f"Tag size is in millimeters, the provided value of {args.size} is prohibited"

    tag_ids = range(args.start_id, args.number)
    generate_tags(aruco_dict, tag_ids, args.size, args.destination)
