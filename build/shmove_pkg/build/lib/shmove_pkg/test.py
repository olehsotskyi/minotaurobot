from create3.nodes import RobotNode
from create3.music import Note

# Initialize the Robot Node
robot = RobotNode()

# Example: Drive forward 1 meter
robot.drive_distance(1.0)  # Distance in meters

# Example: Rotate 90 degrees
robot.rotate_angle(90)  # Angle in degrees

# Example: Set LED colors
robot.set_lights_on_rgb(255, 0, 0)  # Red

# Example: Play a note
robot.play_note(Note.C4, 0.5)  # Play C4 for 0.5 seconds

# Shutdown cleanly
robot.shutdown()