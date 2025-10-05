## TeamCode Module

Welcome!

This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Auto Shooter April Tag Docs

To automatically shoot without manually adjusting Velocity we currently use april tags
We only have the blue alliance goal, so the current April Tag tracking works on only Tag 20

First we setup the limelight object and switch the pipeline to the one that tracks April Tags
    This pipelines need to be setup and tuned using the web interface by connecting limelight to laptop

Next, we pull some data from the limelight: Tx, Ty, Ta, and most importantly Estimated Pose
Tx is the x error of the April Tag (Error is calculated based on center of april tag compared to the center of the camera frame-- this can be changed in the limelight settings but why bother)
Ty is the y error of the April Tag (Calculated similarly as Tx)
Ta is the are of the April Tag (This is computed as percentage of the frame that is covered by the april tag-- if the tag covered like half the frame Ta would ~50%)

BotPose is only available when you turn on 3D tracking in the web interface. 
Once you do this, the limelight will automatically estimate the position of the robot in meters relative to the CENTER OF THE FIELD

Once you have these coordinates, you can translate them to be relative to the goal, relative to starting position (basically however you want)
For example, if the limelight return (0, 0) that means that you are right in the middle of the field. Translated in terms of the goal, you would be about ~(1.5, 1.5)
Be careful with the negatives and your forwards/backwards. The field coordinates are rotated 90 degrees counterclockwise.

Okay now that we have the x and y relative to the goal, we can calculate the distance from the goal using pythagorean theorem.

Now we have distance from the goal
We have control over velocity of the fly wheel
Our shooting angle (theta) is constant
The height of the goal is constant

SO we can use physics equations to find the relationship between distance from the goal and required velocity
