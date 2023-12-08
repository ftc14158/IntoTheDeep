package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ArmConstants {
    public static final String ELBOW_MOTOR1 = "elbowleft";
    public static final String SLIDE_MOTOR = "slide";
    public static final String SLIDE_LIMIT_SWITCH = "slideLimit";
    public static final String WRIST_MOTOR = "wrist";
    public static final String WRIST_POTENTIOMETER = "wristAngle";
    public static final String ELBOW_POTENTIOMETER = "elbowAngle";

    // When arm is at rest/home, encoder position 0, how many degrees it is below horizontal
    // degrees below where gravity is at max
    public static final double ARM_HOME_DEGREES_BELOW_HORIZONTAL = 45; //  // 46;

    public static final double ARM_TICKS_PER_REVOLUTION = 2124;

    // Offset so potentiometer reports 0 degrees when arm in home position
    public static double ARM_POT_HOME_ANGLE_DEGREES = -22.5;

    public static double ARM_HOME_RADS = -ARM_HOME_DEGREES_BELOW_HORIZONTAL * Math.PI / 180.;
    public static double ARM_POS_TO_RADS = (360./ARM_TICKS_PER_REVOLUTION) * (Math.PI / 180.);

    public static double ARM_POWER_SIGN = -1.;

    public static double ARM_KP = 0.9; // 0.0017; // 0.0028; // 0.0005;
    public static double ARM_KI = 0.5; // 0.1; // 0.000;
    public static double ARM_KD = 0.01; // 0.00019; // 0.00028;
    public static double ARM_KS = 0.0;
    public static double ARM_KCOS = 0.15; // 0.15; // 0.45; // 0.18;
    public static double ARM_KCOS_EXT = 0.01;   // multiplier for kcos to slide extent (0->1)

    // Arm actually balances 20 degrees before vertical, so max gravity is 20 degrees below horizontal
    public static double ARM_BALANCE_OFFSET_RADS = 20 * (Math.PI / 180.);

    public static double ARM_KV = -0.09; // 0.0;
    public static double ARM_KA = 0.0;
    // clamp max error from PID controller
    public static double ARM_PCLAMP_HIGH = 1.0; // 0.45;
    public static double ARM_PCLAMP_LOW = -1.0; // 0.45;

    // Preset arm positions as degrees above/below horizontal
    public static double POSITION1 = -10;
    public static double POSITION2 = 5;
    public static double POSITION3 = 105;

    // How many degrees to nudge up/down when using gamepad
    public static double NUDGE_DEGREES = 10;

    // Grab servo degrees
    public static double GRAB_OPEN = 125;
    public static double GRAB_CLOSED = 180;

    public static double DRONE_LAUNCH = 1;
    public static double DRONE_SET = 0.15 ;



}
