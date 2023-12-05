package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogInputController;

/**
 * Wrapper  class for Rev Potentiometer that also:
 *
 * Converts voltage to angle
 * Caches reading since last update() called
 */

public class RevPotentiometer {

    private AnalogInput pot;

    private double volts = -1;  // has not been read

    public RevPotentiometer(AnalogInput pot) {
        this.pot = pot;
    }

    private double getVolts() {
        if (volts < 0) update();
        return volts;
    }

    public double update() {
        volts = pot.getVoltage();
        return volts;
    }

    /**
     * Calculate voltage expected for a given pot taper angle
     * @param degrees
     * @return Expected voltage
     */
    static public double degreesToVoltage(double degrees) {
        // Taken from REV applications document https://docs.revrobotics.com/potentiometer/untitled-1
        return (445d * (degrees - 270d)) / (Math.pow(degrees,2) - 270d*degrees - 36450d );
    }

    static public double radiansToVoltage(double degrees) {
        return degreesToVoltage( Math.toDegrees(degrees) );
    }

    static public double voltageToAngle(double voltage) {
        if (voltage > 0)
            // the inverse of the degrees-to-voltage formula
            return ( ( (2700d*voltage) + 4455d - Math.sqrt(21870000d*Math.pow(voltage,2) - 24057000d*voltage + 19847025d ) ) / (20d*voltage) );

        else return 0;
    }

    static public double voltageToRadians(double voltage) {
        return Math.toRadians(voltageToAngle(voltage));
    }
    public double getVoltage() {
        return getVolts();
    }

    public double getDegrees() {
        return voltageToAngle(getVolts());
    }

    public double getRadians() {
        return voltageToRadians(getVolts());
    }

    public AnalogInput getPot() {
        return pot;
    }
}
