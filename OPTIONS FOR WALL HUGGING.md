If we want to implement a Shortest Route / Wall Hugging behavior, we can inject a wall-bias modifier. Here are two ways we could do it:

## Option 1
Static Inner Wall (If lap direction is known) If the bot always runs the track in a specific direction (e.g., Counter-Clockwise), we can add a persistent bias that always pulls the target angle slightly to the left, hugging the left wall at a safe distance of ~20cm.

## Option 2
Dynamic Corner Cutting We can calculate the distance to the left (90°) and right (-90°) walls in real-time. Whichever wall is closer becomes the "inner" wall. We then subtract a few degrees from t_ang to pull the bot closer to that inner wall, using the danger_sense threshold to ensure it never gets too close.