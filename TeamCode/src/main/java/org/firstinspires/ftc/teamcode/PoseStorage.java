package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class PoseStorage {
    // Static data persists as long as the Robot Controller app runs on the robot
    // (i.e. until Restart Robot or power cycle)
    // so values are retained in betweeen op modes

    // This value is updated at end of OpModes
    public static Pose2d currentPose = new Pose2d(0, 0, 0);


}
