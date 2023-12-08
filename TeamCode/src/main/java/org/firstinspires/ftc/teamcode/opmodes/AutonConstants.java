package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutonConstants {

    public static double APRILTAG_KY = 0.07;
    public static double APRILTAG_KX = 0.1;
    public static double APRILTAG_CLAMP = 0.3;

    public static double APRILTAG_SETPOINT_X = 0.3; // 2.4;
    public static double APRILTAG_SETPOINT_Y = 11.4; // 11.6;
    public static double APRILTAG_SETPOINT_B = -12.0;

    public static long APRILTAG_TIMEOUT = 2500;

    public static int ARM_APRILTAGSCAN_POSITION = -5;
    public static int ARM_RELEASE_POSITION = -15;
    public static double WRIST_RELEASE_POSITION = 60;
    public static double SLIDE_APRILTAGSCAN_POSITION = 2000;
    public static double SLIDE_RELEASE_POSITION = 2400;
}
