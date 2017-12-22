# Modelling, simulation and control of a humanoid robot

## Description
The aim of this project was to develop a controller for a DARWINOP robot that allowed it to intert a pin into a hole. The developed code can be found in the "controllers/PinInHoleController" folder. The other code around this is automaticly generated from Webots.

## Requirements
As previously discussed, the aim of this project was to get the Darwin-op robot to find a pin, pick it up and put it through a hole. In order to complete and develop this project extra specifications needed to be defined. This included assumptions needed and made for this project, for example the colour of the pin and the size of the hole. The full specification list can be found below.

## Specs
* Assume pin is always on the floor (below robot arm level)
* End of the pin should be a distinct colour
* Assume the hole is at arm level
* Hole should be large enough for the Darwin-op robot arm to fit through it
* Pin is within 0.5-meter radius of the robot
* Hole is within 0.5-meter radius of the robot
* 30 min time limit to complete the task (battery length)
* Floor is a block colour i.e. not patterned
* Floor is a different colour from the pin
* Hole has a distinct outline
* Hole is vertical
* Assume ball is the same side of the hole as the robot
* Ball is always red (nothing else can be red)
* Hole always a circle (nothing else can be circular from eye level up)

## Design
Once the use cases were stated it was then possible to start designing the system. The C++ program language was chosen as Webots supports this language and it allows for object orientated program to be used. The code was written in an object orientated design in order to provide modularity and reusability. It also allows for encapsulation allowing classes to hide and protect certain values.

Two main classes were defined; the robot class and the target class. Both the pin and the hole extend the Target class. Additional target items can be added, allowing the robot to find, walk and interact with different targets in different manners. Details of these classes can be seen below in the class diagram.

## Challenges
While writing this software there were a number of challenges and constraints. As can be seen, the arm that Darwin currently has does not allow for objects to be picked up. This was due to being unable to control the custom made gripper within Webots. As a results, when simulating this task, the robot touched the pin instead of picking it up.

Another issue was found when trying to get the Darwin to walk towards the hole. As explained, gray scale was needed in order to blur the image and get circle detection. However, when trying to do this directly in the CircularHole class the controller would crash. As a get around, the image was changed to gray scale in the robot class and then passed across. A number of issues were also found with finding and walking to the hole. The circle detection isn’t that robust. Minor changes to both the x and y axis had to be made in order to keep the circle in the robot’s sight. Additionally, if the robot was at an angle to the hole, the hole would not be detected as it was viewed as an ellipse rather than a circle.

Another the main issues with the hole was knowing when the robot had reached the hole. As expected, as the robot got closer to the hole, the circle appeared bigger. Eventually, the circle became too large to fit in the camera so the robot was no longer able to detect it. Seeing as the Darwin-op does not have any distance sensors, at this point the robot is hardcoded to walk a set number of steps until it reaches the hole.
