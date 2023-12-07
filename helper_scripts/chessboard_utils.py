"""
Chess Robot Movement Path Generator

This script calculates the movement path for a robotic arm designed to move chess pieces on a chessboard. The script reads configuration details such as the board's dimensions and piece height from a YAML file and uses these to generate a 3D path that the robot will follow to move a chess piece from one square to another.

The board is oriented such that the A1 square has the largest y-value and the smallest x-value, while the H8 square has the smallest y-value and the largest x-value. The script takes into account the height of the pieces, ensuring the robot arm moves just below the top of the piece for picking and dropping.

Usage:
Module to output a numpy array representing the robot's movement path.

Author: nranabhat
Date: 12/6/23
"""

import numpy as np
import yaml

# Function to load configuration from a YAML file
def load_config(file_path):
    with open(file_path) as file:
        return yaml.safe_load(file)

# Function to convert chess square to coordinates
def square_to_coord(square, board_width, board_height, origin, piece_height):
    col = ord(square[0].upper()) - ord('A')
    row = int(square[1]) - 1
    square_width = board_width / 8
    square_height = board_height / 8

    x = origin[0] + (row * square_height) + (square_height / 2)
    y = (board_width / 2) - (col * square_width) - (square_width / 2)
    return np.array([x, y, 0])

# Function to create a path for the robot arm
def make_move_path(start, end, config):
    board_width = config['board']['width']
    board_height = config['board']['height']
    origin = np.array([config['origin']['x'], config['origin']['y'], config['origin']['z']])
    piece_height = config['piece']['height']

    start_coord = square_to_coord(start, board_width, board_height, origin, piece_height)
    end_coord = square_to_coord(end, board_width, board_height, origin, piece_height)

    # Adjusting the height to be 1cm below/above the top of the piece
    delta_h = 0.01
    
    # Path points
    path = np.array([
        origin,                                                 # Start at origin
        start_coord + np.array([0, 0, piece_height + delta_h]), # Move above the piece
        start_coord + np.array([0, 0, piece_height - delta_h]), # Lower to the piece
        start_coord + np.array([0, 0, 2 * piece_height + delta_h]), # Lift the piece
        end_coord + np.array([0, 0, 2 * piece_height + delta_h]),   # Move to above the end square
        end_coord + np.array([0, 0, piece_height - delta_h]),       # Lower the piece to end square
        end_coord + np.array([0, 0, piece_height + delta_h]),       # Lift up from the end square
        origin                                                      # Return to origin
    ])

    return path
