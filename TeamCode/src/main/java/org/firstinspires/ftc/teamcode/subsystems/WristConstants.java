package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class WristConstants {
    public static double WRIST_DRIVE_SIGN = 0.4;   // limit on max motor power
    // Wrist angle increases as wrist moves upwards when arm is facing forward
    // Angle reported from wrist pot when wrist in line with arm
    public static double WRIST_POT_LEVEL_ANGLE_DEGREES = 183.0;
    // how close in degrees to consider at set point
    public static double WRIST_DEGREES_TOLERANCE = 4.0;
    // Wrist angles to use for
    public static double WRIST_POS0 = 43.0;
    public static double WRIST_POS1 = 0;
    public static double WRIST_POS2 = 60.0;
    public static double WRIST_KP = 0.02;
    public static double WRIST_KI = 0.1;
    public static double WRIST_KD = 0.0006;
    public static double WRIST_KS = -0.05;
    public static double WRIST_KCOS = 0.12;
}
