# Robotics-Project-2
project 2, basic roomba computer vision + color tracking


PART 1
This project used color tracking to keep a roomba equidistant from some color stimuli. It works with any color.
We establish a distance bewteen the roomba and the blob (a blue or red square held in front of the camera) by
adjusting and taking frequent pictures until the image is aligned in the center. Then we move the roomba accordingly
if there is a shift in where the colored square is. if the colored square moves out of sight too fast, the roomba
should just rotate until it finds it, and then re-establish itself


PART 2
This project involved using a Roomba to find a door in a hallway, then it will knock on 
the door, wait for it to be opened, and enter. Here is what our program does, step by step

1) Orients itself in the center of the hallway calculating the distance between the two walls and
then going halfway

2) rotates 360 degrees taking snapshots and searching for the closest door, storing that orientation

3) moves towards the door by taking frequent pictures and looking for a blob mass of pixels in a binary
image

we used basic gaussian filtering for the color processig
