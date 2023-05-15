

# Maze Message

## Maze Action Explanation
We have two turtlebot waffle and burger. 

1. waffle need to do SLAM and mark down the all airtag location.
2. burger start move from start point.
3. when burger meet the first stop point (S1), it stop and send signal to waffle. 
4. waffle start moving to (W1) location, and tele-operate to push the box.
5. After it done, burger go through the moving box until it reached to S2.
6. After that, waffle move back to origin place (W2);
9. whole process down and robot stop moving. 


## Maze Action Message Explanation 
**waffle.action** 
1. Waffle need desired target PoseStamp.
2. give back feedback and result.


**burger.action** 
1. Burger require command(string) to teleop.
2. give back feedback and result.


**BridgeConnect.action** 
1. S1 and S2 are two number for burger stop
1. W1 and W2 are two target pose for waffle navigation
2. give back feedback and result.
