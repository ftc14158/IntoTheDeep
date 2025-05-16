package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Wrapper  class for Rev Potentiometer that also:
 *
 * Converts voltage to angle
 * Caches reading since last update() called
 * Tracks timestamp (system time in seconds) when reading taken
 * Allows an angle offset to be set
 */

public class RevPotentiometer {
    private static final double MAX_DEGREES = 270d;

    private AnalogInput pot;

    private double timestamp = 0, lastTimeStamp = 0;

    private double lastRadians = 0;

    private double offsetAngle = 0;
    private double offsetRadians = 0;
    private double velRadians = 0;

    private boolean inverted = false;

    private double volts = -1;  // has not been read

    public RevPotentiometer(AnalogInput pot) {
        this.pot = pot;
    }

    private double getVolts() {
        if (volts < 0) {
         //   RobotLog.i("POT force update to get volts");
            update();
        }
        return volts;
    }

    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    public double update() {
        lastTimeStamp = timestamp;
        lastRadians = voltageToRawRadians(volts);
        double lastVolts = volts;

        volts = pot.getVoltage();

        timestamp = (double) System.nanoTime() / 1E9;

        // Calculate the angular velocity in radians if we have a previous reading

        if (lastTimeStamp > 0.05) { // Tolerate floating pt error on 0
            double period = timestamp - lastTimeStamp;
            velRadians = (voltageToRawRadians(volts) - lastRadians) / period;
        //    RobotLog.i("POT: Period=%f, vDelta=%f, velRads=%f, velDeg=%f", period, volts - lastVolts, velRadians, Math.toDegrees(velRadians));
        } else {
        //    RobotLog.i("NO TIMESTAMP - %f, %f", lastTimeStamp, timestamp);
            velRadians = 0;
        }

        return volts;
    }

    /**
     * Calculate voltage expected for a given pot taper angle
     * @param degrees
     * @return Expected voltage
     */
    static public double rawDegreesToVoltage(double degrees) {
        // Taken from REV applications document https://docs.revrobotics.com/potentiometer/untitled-1
        return (445d * (degrees - 270d)) / (Math.pow(degrees,2) - 270d*degrees - 36450d );
    }

    static public double rawRadiansToVoltage(double degrees) {
        return rawDegreesToVoltage( Math.toDegrees(degrees) );
    }

    static public double voltageToRawAngle(double voltage) {
        if (voltage > 0)
            // the inverse of the degrees-to-voltage formula
            return ( ( (2700d*voltage) + 4455d - Math.sqrt(21870000d*Math.pow(voltage,2) - 24057000d*voltage + 19847025d ) ) / (20d*voltage) );

        else return 0;
    }

    static public double voltageToRawRadians(double voltage) {
        return Math.toRadians(voltageToRawAngle(voltage));
    }

    // Return the expected voltage for a given angle
    public double degreesToVoltage(double degrees) {
        if (inverted) degrees = MAX_DEGREES - degrees;
        return rawDegreesToVoltage( degrees - offsetAngle );
    }

    public double getVoltage() {
        return getVolts();
    }

    private double getInvertedDegrees() {
        double angle = voltageToRawAngle(getVolts());
        return inverted ? MAX_DEGREES - angle : angle;
    }

    public double getDegrees() {
        return (offsetAngle + getInvertedDegrees()) % 360.0;
    }

    private double getInvertedRadians() {
        double rads = voltageToRawRadians(getVolts());
        return inverted ? Math.toRadians(MAX_DEGREES) - rads : rads;
    }

    public double getRadians() {
        return (offsetRadians + getInvertedRadians()) % (Math.PI * 2);
    }

    public double radiansPerSecond() {
        return velRadians;
    }

    public double readingTimestamp() { return timestamp; }

    public AnalogInput getPot() {
        return pot;
    }

    public void setOffsetDegrees(double angle) {
        offsetAngle = angle;
        offsetRadians = Math.toRadians(angle);
    }

    public void setOffsetRadians(double radians) {
        offsetRadians = radians;
        offsetAngle = Math.toDegrees(radians);
    }
    public void resetTimestamp() { timestamp = 0; }
}
