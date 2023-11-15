package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final String ELBOW_MOTOR1 = "elbowleft";
    public static final String ELBOW_MOTOR2 = "elbowright";
    public static final String SLIDE_MOTOR = "slide";
    public static final String WRIST_MOTOR = "wrist";

    // When arm is at rest/home, encoder position 0, how many degrees it is below horizontal
    // degrees below where gravity is at max
    public static final double ARM_HOME_DEGREES_BELOW_HORITZONTAL = 40; //  // 46;

    public static final double ARM_TICKS_PER_REVOLUTION = 2124;

    public static double ARM_HOME_RADS = -ARM_HOME_DEGREES_BELOW_HORITZONTAL * Math.PI / 180.;
    public static double ARM_POS_TO_RADS = (360./ARM_TICKS_PER_REVOLUTION) * (Math.PI / 180.);

    public static double ARM_POWER_SIGN = -1.;

    public static double ARM_KP = 0.0017; // 0.0028; // 0.0005;
    public static double ARM_KI = 0.05; // 0.1; // 0.000;
    public static double ARM_KD = 0.00019; // 0.00028;
    public static double ARM_KS = 0.0;
    public static double ARM_KCOS = 0.27; //  0.45; // 0.18;
    public static double ARM_KCOS_EXT = 0.00012;   // multiplier for kcos at full length

    public static double ARM_KV = 0; // 0.0;
    public static double ARM_KA = 0.0;
    public static double ARM_PCLAMP = 0.45;
    public static double POSITION1 = 330;
    public static double POSITION2 = 530;
    public static double POSITION3 = 860; // 1000;

    public static double GRAB_OPEN = 125;
    public static double GRAB_CLOSED = 180;

    public static double SLIDE_KP = 0.011;
    public static double SLIDE_KI = 0.09;
    public static double SLIDE_KD = 0.0006;
    public static double SLIDE_POWER_SIGN = -0.5;
    public static double SLIDE_TOLERANCE = 15.0;
    public static double SLIDE_MAX = 3550;
    public static double SLIDE_GROUND = 2300;
    public static double SLIDE_CRUISE = 30;
    public static double SLIDE_PIXEL_HEIGHT = 300;
    public static double WRIST_POS1 = 15;
    public static double WRIST_DRIVE_SIGN = -0.35;
    public static double WRIST_KP = -0.1;

}
