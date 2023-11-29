package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

/** Class to control the arm wrist, used by the arm subsystem
 *
 */
public class WristController {
    private AnalogInput anglePot;
    private MotorEx motor;

    private PIDFController controller = null;

    private double anglePotVoltage;

    private double lastError = 0;
    private double setPoint = 0;

    private boolean idle = true;

    public WristController(HardwareMap hardwareMap) {
        idle = true;

        motor = new MotorEx(hardwareMap, ArmConstants.WRIST_MOTOR);
        anglePot = hardwareMap.analogInput.get( ArmConstants.WRIST_POTENTIOMETER );

        motor.stopMotor();
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.RawPower);

        controller = new PIDFController(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD, ArmConstants.WRIST_KF);

    }

    /**
     * Calculate voltage expected for a given pot taper angle
     * @param degrees
     * @return Expected voltage
     */
    private double angleToVoltage(double degrees) {
        // Taken from REV applications document https://docs.revrobotics.com/potentiometer/untitled-1
        return (445d * (degrees - 270d)) / (Math.pow(degrees,2) - 270d*degrees - 36450d );
    }

    private double voltageToAngle(double voltage) {
        if (voltage > 0)
            return ( ( (2700d*voltage) + 4455d - Math.sqrt(21870000d*Math.pow(voltage,2) - 24057000d*voltage + 19847025d ) ) / (20d*voltage) );
        else return 0;
    }

    public void update() {
        anglePotVoltage = anglePot.getVoltage();

        // apply controller
        if (idle) {
            lastError = 0;
            motor.stopMotor();
        } else {
            double error = controller.calculate(anglePotVoltage);
            error = com.arcrobotics.ftclib.util.MathUtils.clamp(error, -1.0, 1.0);
            lastError = error;
            motor.set(ArmConstants.WRIST_DRIVE_SIGN * error);
        }
    }

    /**
     * Return current angle of wrist relative to arm
     * using last sensor reading
     *
     * Formula is the formula from the REV applications document solved for angle
     */

    public double angle() {
        return voltageToAngle(anglePotVoltage);
    }


    public double voltage() {
        return anglePotVoltage;
    }

    public double error() {
        return lastError;
    }

    public double setPoint() { return setPoint; }

    public boolean inPosition() {

        return idle || ( Math.abs( voltageToAngle(setPoint) - voltageToAngle(anglePotVoltage) ) < ArmConstants.WRIST_DEGREES_TOLERANCE );
    }

    /**
     * Reset the PIDF controller using current constants, and given setpoint
     * @param angleDegrees
     */
    private void setController(double angleDegrees) {
        Log.i("***WRIST***", "Set angle " + angleDegrees);
        setPoint = angleToVoltage( angleDegrees );
        controller.setPIDF(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD, ArmConstants.WRIST_KF);
        controller.setSetPoint( angleToVoltage(angleDegrees) );
        idle = false;
    }

    public void setRelativeAngle(double angleDegrees) {

        setController( ArmConstants.WRIST_POT_LEVEL_ANGLE_DEGREES + angleDegrees );
        idle = false;
    }

    public void stop() {
        idle = true;
    }
}
