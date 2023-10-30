package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final String ELBOW_MOTOR1 = "elbowleft";
    public static final String ELBOW_MOTOR2 = "elbowright";
    public static final String SLIDE_MOTOR = "slide";
    public static final String WRIST_MOTOR = "wrist";

    // When arm is at rest/home, encoder position 0, how many degrees it is below horizontal
    public static final double ARM_HOME_DEGREES_BELOW_HORITZONTAL = 0; // 46;

    public static final double ARM_TICKS_PER_REVOLUTION = 8192;

    public static double ARM_HOME_RADS = -ARM_HOME_DEGREES_BELOW_HORITZONTAL * Math.PI / 180.;
    public static double ARM_POS_TO_RADS = (360./ARM_TICKS_PER_REVOLUTION) * (Math.PI / 180.);

    public static double ARM_POWER_SIGN = -1.;

    public static double ARM_KP = 0.00065; // 0.0005;
    public static double ARM_KI = 0.07; // 0.000;
    public static double ARM_KD = 0.000;
    public static double ARM_KF = 0.000;
    public static double ARM_KS = 0.0;
    public static double ARM_KCOS = 0.09; // 0.18;
    public static double ARM_KV = -0.2; // 0.0;
    public static double ARM_KA = 0.0;
    public static double POSITION1 = 400;
    public static double POSITION2 = 900;
    public static double POSITION3 = 1400;
}
