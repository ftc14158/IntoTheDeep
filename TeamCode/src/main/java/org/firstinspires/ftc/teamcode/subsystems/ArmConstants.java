package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final String ELBOW_MOTOR1 = "elbowleft";
    public static final String SLIDE_MOTOR = "slide";
    public static final String SLIDE_LIMIT_SWITCH = "slideLimit";
    public static final String WRIST_MOTOR = "wrist";
    public static final String WRIST_POTENTIOMETER = "wristAngle";

    // When arm is at rest/home, encoder position 0, how many degrees it is below horizontal
    // degrees below where gravity is at max
    public static final double ARM_HOME_DEGREES_BELOW_HORIZONTAL = 40; //  // 46;

    public static final double ARM_TICKS_PER_REVOLUTION = 2124;

    public static double ARM_HOME_RADS = -ARM_HOME_DEGREES_BELOW_HORIZONTAL * Math.PI / 180.;
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
    public static double POSITION3 = 900;

    // Grab servo degrees
    public static double GRAB_OPEN = 125;
    public static double GRAB_CLOSED = 180;

    // Slide PID co-efficients
    public static double SLIDE_KP = 0.015;
    public static double SLIDE_KI = 0.3;
    public static double SLIDE_KD = 0.0006;

    // The max power to be applied to the slide motor
    public static double SLIDE_POWER_SIGN = -0.7;

    // Power level to use when moving slide back to find home
    public static double SLIDE_HOME_POWER = 0.35;

    // Hoist power when lifting
    public static double SLIDE_HOIST_POWER = 1.0;

    // How close slide position must be before stopping
    public static double SLIDE_TOLERANCE = 5.0;

    // Max slide position
    public static double SLIDE_MAX = 3550;

    // Slide position when grabber at ground
    public static double SLIDE_GROUND = 2200;

    // Slide position when arm level for driving
    public static double SLIDE_CRUISE = 30;

    // Slide distance to raise by one pixel height
    public static double SLIDE_PIXEL_HEIGHT = 240;

    // Wrist angle increases as wrist moves upwards when arm is facing forward
    // Angle reported from wrist pot when wrist in line with arm
    public static double WRIST_POT_LEVEL_ANGLE_DEGREES = 88.0;

    // how close in degrees to consider at set point
    public static double WRIST_DEGREES_TOLERANCE = 4.0;

    // Wrist angles to use for
    public static double WRIST_POS0 = -44.0;
    public static double WRIST_POS1 = 0;
    public static double WRIST_POS2 = -60.0;

    public static double WRIST_DRIVE_SIGN = 0.3;   // clamp on max motor speed
    public static double WRIST_KP = 4;
    public static double WRIST_KI = 0.8;
    public static double WRIST_KD = 0.05;
    public static double WRIST_KF = 0.2;

    public static double DRONE_LAUNCH = 1;
    public static double DRONE_SET = 0.15 ;



}
