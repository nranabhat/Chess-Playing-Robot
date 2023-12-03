import numpy as np
import yaml

# Load constants from the YAML file
with open('robot.yaml') as file:
    config = yaml.safe_load(file)

board_width = config['board']['width']
board_height = config['board']['height']
origin = np.array([config['origin']['x'], config['origin']['y'], config['origin']['z']])
piece_height = config['piece']['height']

# Function to convert chess square to coordinates
def square_to_coord(square):
    col = ord(square[0].upper()) - ord('A')
    row = 8 - int(square[1])
    square_width = board_width / 8
    square_height = board_height / 8
    x = (col * square_width) + (square_width / 2)
    y = (row * square_height) + (square_height / 2)
    return np.array([x, y, 0])

# Function to create the movement path
def make_move_path(start, end):
    start_coord = square_to_coord(start)
    end_coord = square_to_coord(end)

    # Adjusting the height to be 1cm below/above the top of the piece
    grip_surface_height = 0.01
    # have to robot transfer pieces so that it doesn't knock over other pieces.
    
    # Path points
    above_start = start_coord + np.array([0, 0, piece_height+grip_surface_height])
    grip_start = above_start - np.array([0, 0, 2*grip_surface_height])
    begin_transfer = above_start + np.array([0, 0, piece_height])
    end_transfer = end_coord + np.array([0, 0, piece_height])
    grip_end = end_coord - np.array([0, 0, piece_height+2*grip_surface_height])
    above_end = grip_end + np.array([0, 0, 2*grip_surface_height])

    path = [
        origin,                   # Start at origin
        above_start,              # Move above the piece
        grip_start,               # Lower to the piece
        begin_transfer,           # Lift the piece above the other ones
        above_end,                # Move to above the end square
        end_transfer,             # Lower the piece to end square
        above_end,                # Lift up from the end square
        origin                    # Return to origin
    ]

    return np.array(path)

# Main execution
if __name__ == "__main__":
    start_square = input("Enter the starting square (e.g., A7): ")
    end_square = input("Enter the ending square (e.g., D3): ")

    move_path = make_move_path(start_square, end_square)
    print("Path for the robot to follow:")
    print(move_path)
