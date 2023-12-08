package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.MechanismMode;
import org.firstinspires.ftc.teamcode.subsystems.WristConstants;

import java.util.HashMap;
import java.util.Map;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.*;

/** Class to control the arm wrist, used by the arm subsystem
 *
 */
public class WristController {
    private RevPotentiometer anglePot;
    private MotorEx motor;

    private PIDController controller = null;
    private ArmFeedforward ffController = null;

    private double anglePotVoltage;

    private double lastError = 0;
    private double setPoint = 0;

    private double error, ff;

    private double offsetDegrees = 0;

    private boolean setPointRelative;

    private MechanismMode mode = IDLE;

    public WristController(HardwareMap hardwareMap) {
        mode = IDLE;

        motor = new MotorEx(hardwareMap, ArmConstants.WRIST_MOTOR);
        anglePot = new RevPotentiometer(hardwareMap.analogInput.get( ArmConstants.WRIST_POTENTIOMETER ));
        anglePot.setInverted(true);

        motor.stopMotor();
        motor.setInverted(true);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setRunMode(Motor.RunMode.RawPower);

        controller = new PIDController(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        ffController = new ArmFeedforward(WristConstants.WRIST_KS, WristConstants.WRIST_KCOS, 0);
    }

    public void update() {
        anglePot.update();  // read the current actual value

        // apply controller
        if (mode == IDLE) {
            lastError = 0;
            motor.stopMotor();
        } else {
            updateControllerSetpoint();

            error = controller.calculate( angle() );
            error = clamp(error, -1.0, 1.0);

            // Calculate the feedforward for gravity
            // get angle relative to ground
            double groundAngleRads = Math.toRadians(offsetDegrees + getRelativeAngle());
            ff = ffController.calculate(groundAngleRads, anglePot.radiansPerSecond(), 0 );

            lastError = error + ff;

            motor.set(WristConstants.WRIST_DRIVE_SIGN * lastError);
        }
    }

    public void setOffsetDegrees(double degrees) {
        offsetDegrees = degrees;
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

        return (mode == IDLE) || ( ( setPoint - angle() ) < WristConstants.WRIST_DEGREES_TOLERANCE );
    }

    /**
     * Reset the PIDF controller using current constants, and given setpoint
     * @param angleDegrees
     */
    private void setController(double angleDegrees, boolean relative) {
        setPoint = angleDegrees;
        setPointRelative = relative;

        controller.setPID(WristConstants.WRIST_KP, WristConstants.WRIST_KI, WristConstants.WRIST_KD);
        ffController = new ArmFeedforward(WristConstants.WRIST_KS, WristConstants.WRIST_KCOS, 0);

        updateControllerSetpoint();
        mode = POSITIONING;
    }

    private void updateControllerSetpoint() {
        double calcedSetpoint = setPoint;
        if (!setPointRelative) calcedSetpoint -= offsetDegrees;
        if ( Math.abs(controller.getSetPoint() - calcedSetpoint ) > 0.1 ) controller.setSetPoint( calcedSetpoint );
    }

    /**
     * Set wrist angle relative to in line with the arm
     *
     * WRIST_POT_LEVEL_ANGLE_DEGREES is the angle reported by the pot when the
     * wrist is in line with the arm
     *
     * A negative angleDegrees will raise wrist above the inline angle
     */

    public void setRelativeAngle(double angleDegrees) {
        setController( WristConstants.WRIST_POT_LEVEL_ANGLE_DEGREES + angleDegrees, true );
    }

    public void setAngleAboveHorizontal(double angleDegrees) {
        setController( WristConstants.WRIST_POT_LEVEL_ANGLE_DEGREES + angleDegrees, false );
    }

    public double getRelativeAngle() {
        return angle() - WristConstants.WRIST_POT_LEVEL_ANGLE_DEGREES;
    }

    public void stop() {
        mode = IDLE;
    }

    public Map<String, Object> debugInfo() {
        Map<String, Object> data = new HashMap<String, Object>();

        data.put("Mode", mode);
        data.put("Angle", getRelativeAngle() );
        data.put("Ground angle", getRelativeAngle() + offsetDegrees);
        data.put("Voltage", voltage());
        data.put("PID Error", error );
        data.put("Feedforward", ff );
        data.put("Setpoint (rel angle)", setPoint() - WristConstants.WRIST_POT_LEVEL_ANGLE_DEGREES );
        return data;
    }
}
