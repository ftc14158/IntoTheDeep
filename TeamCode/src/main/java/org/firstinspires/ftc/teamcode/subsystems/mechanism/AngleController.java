package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import com.arcrobotics.ftclib.controller.PController;
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
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.IDLE;
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.POSITIONING;
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.RETURNING_TO_HOME;
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.SETTLING_HOME;

/**
 * Control the main arm angle
 *
 */
public class AngleController {

    private RevPotentiometer anglePot;
    private MotorEx motor;

    private PController pController = null;
    private ArmFeedforward ffController = null;

    private MechanismMode mode = IDLE;
    private double anglePotVoltage;

    private double lastPError = 0;
    private double lastFFError = 0;
    private double lastPower = 0;

    private double setPoint = 0;

    // arm extended proportion from 0 to 1
    private double extension = 0;

    private int iSteady = 0;

    private boolean potZeroAngleSet = false;

    public AngleController(HardwareMap hardwareMap) {
        mode = IDLE;

        motor = new MotorEx(hardwareMap, ArmConstants.ELBOW_MOTOR1);

        anglePot = new RevPotentiometer(hardwareMap.analogInput.get(ArmConstants.ELBOW_POTENTIOMETER));
        anglePot.setOffsetDegrees( ArmConstants.ARM_POT_HOME_ANGLE_DEGREES );

        motor.stopMotor();
        motor.setInverted(false);
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.resetEncoder();
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.setDistancePerPulse(1);
        motor.setPositionCoefficient(0);

        pController = new PController(1);
        ffController = new ArmFeedforward(.1, .1, 0);
    }


    public void update() {
        double lastPositionRadians =  anglePot.getRadians();

        anglePot.update();  // read the current actual value
        double currentPositionRadians = anglePot.getRadians();

        if (lastPositionRadians == 0) {
            lastPositionRadians = currentPositionRadians;
        }

        // add the home angle to the pot reported angle
        // so that angle of zero radians = arm level

        currentPositionRadians += ArmConstants.ARM_HOME_RADS;
        lastPositionRadians += ArmConstants.ARM_HOME_RADS;

        // The arm feedforward controller needs the arm position in radians, so we convert the
        // position in tick to radians roughly here. Assumes that when arm is raised to horizontal,
        // the encoder position is 400. And when arm is resting (not quite straight down), encoder
        // position is zero. So if arm was actually straight down, encoder position would be -100

        // the arm position should be 0 degrees when horizontal requiring maximum position power to
        // raise it

        switch(mode) {
            case POSITIONING:
            case RETURNING_TO_HOME:
                // Velocity on the encoder is ticks/sec - pos = upwards, 8192 ticks = 360 degrees

                // ticks per sec to rads per sec
                lastPError = pController.calculate(currentPositionRadians);
                // clamp p_error to a max value
                double clampedPError = clamp(lastPError, ArmConstants.ARM_PCLAMP_LOW, ArmConstants.ARM_PCLAMP_HIGH);
                lastPError = clampedPError;

                // The balance gets closer to vertical as arm extends..


                double balanceAdjustedRadians = setPoint + ( ArmConstants.ARM_BALANCE_OFFSET_RADS * (1.2 - extension) );

                lastFFError =  ffController.calculate(balanceAdjustedRadians, anglePot.radiansPerSecond() );
                // increase kcos error based on slide length
                lastFFError += ( (ArmConstants.ARM_KCOS_EXT * extension) * Math.cos(balanceAdjustedRadians) );

                // Precaution: Turn off motor once arm is within 5 degrees of home to avoid controller taking too long to
                // bring arm to home position. Major problem if arm does not home accurately and quickly.
                if (mode == RETURNING_TO_HOME && ( (currentPositionRadians - ArmConstants.ARM_HOME_RADS ) < Math.toRadians(5))) {
                    // within 3 degrees  of home, so drop the rest of the way with gravity
                    // and turn off motor
                    setPower(0);
                    mode = SETTLING_HOME;
                } else {
                    setPower( clampedPError + lastFFError );
                }
                break;

            case SETTLING_HOME:
                // If less than 0.25 degrees movement for 6 consecutive cycles, assume at home position
                if ( Math.abs(Math.toDegrees(currentPositionRadians - lastPositionRadians)) < 0.25 ) {
                    iSteady++;
                } else {
                    iSteady = 0;
                }

                if (iSteady > 4) {
                    if (iSteady == 5) {
                        mode = IDLE;
                    }
                    iSteady = 6;
                }
                break;

            case IDLE:
                // in idle mode,
                iSteady = 0;
                motor.stopMotor();

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
        return lastPError + lastFFError;
    }

    public double setPoint() { return setPoint; }

    public boolean inPosition() {
        return (mode == IDLE) || ( ( setPoint - angle() ) < WristConstants.WRIST_DEGREES_TOLERANCE );
    }

    // Set arm extension to increase gravity factor
    public void setExtension(double extension) {
        this.extension = extension;
    }

    /**
     * Reset the PIDF controller using current constants, and given setpoint
     * @param angleRadians - target angle relative to horizontal
     */
    private void setController(double angleRadians) {
        setPoint = angleRadians;
        anglePot.resetTimestamp();
        pController.setPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD );
        pController.reset();
        ffController = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KCOS, ArmConstants.ARM_KV, ArmConstants.ARM_KA);

        pController.setSetPoint( angleRadians);
    }

    // Move arm to a number of degrees above/below horizontal

    public void setDegreesAboveHorizontal(double degrees ) {
        setController( Math.toRadians( degrees ) );
        mode = POSITIONING;
    }

    // lower arm to starting/home position
    public void returnToHome() {
        setController( Math.toRadians( - ArmConstants.ARM_HOME_DEGREES_BELOW_HORIZONTAL ) );
        mode = RETURNING_TO_HOME;
    }

    public double getSetpointDegrees() {
        return Math.toDegrees(setPoint);
    }

    public double getDegreesAboveHorizontal() {
        return angle() - ArmConstants.ARM_HOME_DEGREES_BELOW_HORIZONTAL;
    }

    public void stop() {
        mode = IDLE;
    }

    public MechanismMode getMode() {
        return mode;
    }

    public void adjustSetpoint(double degrees) {
        if (mode == POSITIONING) {
            double newSetpoint = clamp( getSetpointDegrees() + degrees, -ArmConstants.ARM_HOME_DEGREES_BELOW_HORIZONTAL, ArmConstants.POSITION3 );
            setDegreesAboveHorizontal(newSetpoint);
        }
    }

    public Map<String, Object> debugInfo() {
        Map<String, Object> data = new HashMap<String, Object>();

        data.put("Mode", mode);
        data.put("Arm Angle", Math.toDegrees(anglePot.getRadians() + ArmConstants.ARM_HOME_RADS) );
        data.put("Pot Angle", angle() );
        data.put("Setpoint", Math.toDegrees(setPoint()));
        data.put("PError", lastPError );
        data.put("FFError", lastFFError );
        data.put("Power", lastPower);
        data.put("Voltage", voltage());
        return data;
    }

    private void setPower(double power)
    {
        lastPower = power;
        motor.set(power * ArmConstants.ARM_POWER_SIGN);
    }
}
