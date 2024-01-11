package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutonConstants {

    public static double APRILTAG_KY = 0.06; // 0.07;
    public static double APRILTAG_KX = 0.06; // 0.1;
    public static double APRILTAG_CLAMP = 0.3;

    public static double APRILTAG_SETPOINT_X_LEFT = 2.4; // 0.3; // 2.4;
    public static double APRILTAG_SETPOINT_Y_LEFT = 11.6; // 11.4; // 11.6;
    public static double APRILTAG_SETPOINT_X_RIGHT = -0.6; // 0.3; // 2.4;
    public static double APRILTAG_SETPOINT_Y_RIGHT = 11.6; // 11.4; // 11.6;
    public static double APRILTAG_SETPOINT_B_LEFT = -10.0;
    public static double APRILTAG_SETPOINT_B_RIGHT = 7.0;

    public static double APRILTAG_TIMEOUT = 2800;

    public static int ARM_APRILTAGSCAN_POSITION = -5;
    public static int ARM_RELEASE_POSITION = -13;
    public static double WRIST_RELEASE_POSITION = 65;
    public static double SLIDE_APRILTAGSCAN_POSITION = 2000;
    public static double SLIDE_RELEASE_POSITION = 3100; // 2400;

    public static double PIXEL_RELEASE_DELAY_MS = 500;

}
