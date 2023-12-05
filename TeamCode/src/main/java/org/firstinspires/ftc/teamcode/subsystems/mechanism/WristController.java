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
import org.firstinspires.ftc.teamcode.subsystems.MechanismMode;

import java.util.HashMap;
import java.util.Map;

import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.*;

/** Class to control the arm wrist, used by the arm subsystem
 *
 */
public class WristController {
    private RevPotentiometer anglePot;
    private MotorEx motor;

    private PIDFController controller = null;

    private double anglePotVoltage;

    private double lastError = 0;
    private double setPoint = 0;

    private MechanismMode mode = IDLE;

    public WristController(HardwareMap hardwareMap) {
        mode = IDLE;

        motor = new MotorEx(hardwareMap, ArmConstants.WRIST_MOTOR);
        anglePot = new RevPotentiometer(hardwareMap.analogInput.get( ArmConstants.WRIST_POTENTIOMETER ));

        motor.stopMotor();
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.RawPower);

        controller = new PIDFController(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD, ArmConstants.WRIST_KF);

    }

    public void update() {
        anglePot.update();  // read the current actual value

        // apply controller
        if (mode == IDLE) {
            lastError = 0;
            motor.stopMotor();
        } else {
            double error = controller.calculate(anglePot.getVoltage());
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
        return anglePot.getDegrees();
    }

    public double voltage() {
        return anglePot.getVoltage();
    }

    public double error() {
        return lastError;
    }

    public double setPoint() { return setPoint; }

    public boolean inPosition() {

        return (mode == IDLE) || ( Math.abs( RevPotentiometer.voltageToAngle(setPoint) - angle() ) < ArmConstants.WRIST_DEGREES_TOLERANCE );
    }

    /**
     * Reset the PIDF controller using current constants, and given setpoint
     * @param angleDegrees
     */
    private void setController(double angleDegrees) {
        setPoint = RevPotentiometer.degreesToVoltage( angleDegrees );
        controller.setPIDF(ArmConstants.WRIST_KP, ArmConstants.WRIST_KI, ArmConstants.WRIST_KD, ArmConstants.WRIST_KF);
        controller.setSetPoint( RevPotentiometer.degreesToVoltage( angleDegrees) );
        mode = POSITIONING;
    }

    public void setRelativeAngle(double angleDegrees) {

        setController( ArmConstants.WRIST_POT_LEVEL_ANGLE_DEGREES + angleDegrees );
    }

    public void stop() {
        mode = IDLE;
    }

    public Map<String, Object> debugInfo() {
        Map<String, Object> data = new HashMap<String, Object>();

        data.put("Mode", mode);
        data.put("Angle", angle() );
        data.put("Voltage", voltage());
        data.put("Error", error() );
        data.put("Setpoint", setPoint());
        return data;
    }
}
