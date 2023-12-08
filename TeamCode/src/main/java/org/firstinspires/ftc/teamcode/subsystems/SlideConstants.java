package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideConstants {
    // Slide PID co-efficients
    public static double SLIDE_KP = 0.015;
    public static double SLIDE_KI = 0.2;
    public static double SLIDE_KD = 0.0003;
    // The max power to be applied to the slide motor
    public static double SLIDE_POWER_SIGN = -0.7;
    // Power level to use when moving slide back to find home
    public static double SLIDE_HOME_POWER = 0.3;
    // Hoist power when lifting
    public static double SLIDE_HOIST_POWER = 1.0;
    // How close slide position must be before stopping
    public static double SLIDE_TOLERANCE = 5.0;
    // Max slide position
    public static double SLIDE_MAX = 3550;
    // Slide position when grabber at ground
    public static double SLIDE_GROUND = 2050;
    // Slide position when arm level for driving
    public static double SLIDE_CRUISE = 30;
    // Slide distance to raise by one pixel height
    public static double SLIDE_PIXEL_HEIGHT = 300;
}
