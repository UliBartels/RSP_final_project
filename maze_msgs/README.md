We have two turtlebot waffle and burger. 


1. waffle need to do SLAM and mark down the all airtag location.
2. burger start move from start point.
3. when burger meet the first airtag, it stop and send signal to waffle. 
4. waffle start moving to first airtag location.
5. after it done, burger move through waffle until it see second tag.
6. once burger see the second tag, send the signal to waffle and waffle move to second airtag location. 

7. burger go through bridge, and keep going.
8. until it see the desination, BridgeConnect action done.
9. whole process down and robot stop moving.  



Explaintion for action msg of each robot
**waffle.action** 
1. Waffle need signal of airtag number
2. give back feeback  and result.


**burger.action** 
1. burger send signal of airtag to waffle 
2. burger receive the signal of when waffle done the SLAM
3.  
4. give back feeback  and result.

