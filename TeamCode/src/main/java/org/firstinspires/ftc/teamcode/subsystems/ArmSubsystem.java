package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem that operates the robot arm. The aim of this subsystem is to position the arm at either
 * the bottom (resting) position, or one of three levels corresponding to the three levels of the
 * shipping hub, so that an item can be ejected from the intake onto that level.
 *
 * It does this by putting the arm motor in raw power mode, but using a P controller and an
 * ArmFeedForward controller to calculate the error in the position and set continuously set
 * the motor power to go to and hold the target position
 *
 * For resting position, we just turn the motor off and let arm fall back to bottom stop point with
 * gravity.
 */

public class ArmSubsystem extends SubsystemBase {

    private final static boolean DEBUG = false;

    public enum arm_mode { POSITIONING, RETURNING_TO_HOME, SETTLING_HOME, IDLE }

    // These two motors are paired
    private MotorEx elbowMotor1;
    private MotorEx elbowMotor2;

    private MotorEx slideMotor;

    private RobotContainer m_robot;

    private arm_mode m_mode;

    private int m_iTargetPosition = 0;

    private ArmFeedforward m_positionController;
    private PController m_PController;

    private int lastPosition = 0;
    private String lastMsg = "";
    private int iSteady;

    public ArmSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        m_robot = robot;

        m_PController = new PController(1);
        m_positionController = new ArmFeedforward( .1, .1, 0 );

        elbowMotor1 = new MotorEx(hardwareMap, ArmConstants.ELBOW_MOTOR1);
        elbowMotor2 = new MotorEx(hardwareMap, ArmConstants.ELBOW_MOTOR2);
        slideMotor = new MotorEx(hardwareMap, ArmConstants.SLIDE_MOTOR);

        elbowMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        elbowMotor1.setInverted(true);
        elbowMotor1.resetEncoder();
        elbowMotor1.setRunMode(Motor.RunMode.RawPower);
        elbowMotor1.setDistancePerPulse(1);
        elbowMotor1.setPositionCoefficient( 0 ); // 0.02 );

//        elbowMotor2.setInverted(true);
        elbowMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        elbowMotor2.resetEncoder();
        elbowMotor2.setRunMode(Motor.RunMode.RawPower);
        elbowMotor2.setDistancePerPulse(1);
        elbowMotor2.setPositionCoefficient( 0 ); // .02 );

        slideMotor.resetEncoder();
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        slideMotor.setRunMode(Motor.RunMode.RawPower);

        m_mode = arm_mode.IDLE;
        iSteady = 0;

    }

    /**
     * Set the subsystem into positioning mode (so it will use the controllers to try and
     * keep the motor holding a certain position)
     *
     * @param position  The position to move arm to (in encoder ticks)
     */
    public void goToPosition(int position) {

        m_PController.setPIDF(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD, ArmConstants.ARM_KF );
        m_positionController = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KCOS, ArmConstants.ARM_KV, ArmConstants.ARM_KA);

        Log.w("ARM", "Going to position " + position);
//        armMotor.setRunMode(Motor.RunMode.PositionControl );
        // armMotor.setPositionTolerance( 0 );
        m_PController.setSetPoint(position);
        m_iTargetPosition = position;
      //  elbowMotor1.setTargetPosition(m_iTargetPosition);
      //  elbowMotor2.setTargetPosition(m_iTargetPosition);
//        armMotor.set(1);
        m_mode = arm_mode.POSITIONING;
    }

    @Override
    public void periodic() {
        int currentPosition = elbowMotor1.getCurrentPosition();
        String msg = "";
        // max gravity should be when cos(position in radians) = 1
        // so cos(0) = 1, so 0 degrees is horizontal

        // position 400 = horizonal, -100 = vertical down

        // The arm feedforward controller needs the arm position in radians, so we convert the
        // position in tick to radians roughly here. Assumes that when arm is raised to horizontal,
        // the encoder position is 400. And when arm is resting (not quite straight down), encoder
        // position is zero. So if arm was actually straight down, encoder position would be -100

        // the arm position should be 0 degrees when horizontal requiring maximum position power to
        // raise it

        double currentPositionRadians =  ArmConstants.ARM_HOME_RADS + ArmConstants.ARM_POS_TO_RADS * currentPosition;

        double vel = elbowMotor1.getVelocity();
        double accel = elbowMotor1.getAcceleration();
        double accelRads = accel * ArmConstants.ARM_POS_TO_RADS;
        double velRads = vel * ArmConstants.ARM_POS_TO_RADS;

        m_robot.addTelem("Arm Pos", currentPosition);
        m_robot.addTelem("Arm Pos Deg", Math.toDegrees(currentPositionRadians) );
        m_robot.addTelem("Arm Mode", m_mode);

        switch(m_mode) {
            case POSITIONING:
            case RETURNING_TO_HOME:
                // Velocity on the encoder is ticks/sec - pos = upwards, 8192 ticks = 360 degrees

                m_robot.addTelem("PID Setpoint", m_PController.getSetPoint());
                // ticks per sec to rads per sec
                double p_error = m_PController.calculate(currentPosition);
                double ff_error =  m_positionController.calculate(currentPositionRadians, velRads);
                if (DEBUG) msg = ", vel = " + vel + ", p error = " + p_error + ", ff = " + ff_error;
                m_robot.addTelem("ArmPID Vel rad/sec", vel );
                m_robot.addTelem("ArmPID Error", p_error );
                m_robot.addTelem("ArmFF", ff_error);
                if (m_mode == arm_mode.RETURNING_TO_HOME && currentPosition < 100) {
                    // close to home, so drop the rest of the way with gravity
                    setElbowPower(0);
                    m_mode = arm_mode.SETTLING_HOME;
                } else {
                    m_robot.addTelem("Requested power", p_error+ff_error);
                    setElbowPower( p_error + ff_error );
                }
                break;

            case SETTLING_HOME:
                // if not positioning, arm should be dropping to ground/home position.
                // When arm does go back to starting position, the encoder count is never quite
                // back to where it was the last time arm was at the start position because of
                // slippage etc.
                //
                // So, once encoder value appears to no longer be changing for at least 4
                // periods of the loop, assume we are back at the start position, and reset the
                // encoder value back to zero.
                //
                // This is a software alternative to adding a switch to detect when the arm is
                // back at the start position (which would probably be more reliable)
                if (currentPosition == lastPosition) { iSteady++; } else { iSteady = 0; }
                if (iSteady > 4) {
                    if (iSteady == 5) {
                        elbowMotor1.resetEncoder();
                        elbowMotor2.resetEncoder();
                        m_mode = arm_mode.IDLE;
                    }
                    iSteady = 6;
                }
                break;

            case IDLE:
                // in idle mode,
                iSteady = 0;

    //                currentPosition = armMotor.getCurrentPosition();
        }

//        m_telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        if (DEBUG) {
            msg =
                    "Position = "
                            + currentPosition
                            + ", last pos = " + lastPosition
                            + ", power = " + elbowMotor1.get()
                            + ", At position = "
                            + (elbowMotor1.atTargetPosition() ? "YES" : "NO")
                            + ", mode = " + m_mode
                            + ", kcos = " + Math.cos(currentPositionRadians) + msg;
            if (!msg.equals(lastMsg)) {
                lastMsg = msg;
                Log.i("ARM", msg);
            }
        }

        m_robot.addTelem("Elbow Power", elbowMotor1.get());

        lastPosition = currentPosition;

    }

    public int getPosition() {
        return lastPosition;
    }

    /**
     * Turn off positioning mode and just set the arm motor on at a given raw power
     *
     * This is really only for manual control or testing, or for setting the power to
     * zero to let the arm fall back to starting position.
     *
     * @param power
     */
    public void setPower(double power) {
        if (m_mode == arm_mode.IDLE) {
//        elbowMotor1.setRunMode(Motor.RunMode.RawPower);
//        elbowMotor2.setRunMode(Motor.RunMode.RawPower);
            //       if (DEBUG) Log.w("ARM", "Setting power to " + power );
            setElbowPower(power);
        }
    }
    private void setElbowPower(double power) {
        elbowMotor1.set(power * ArmConstants.ARM_POWER_SIGN);
        elbowMotor2.set(power * ArmConstants.ARM_POWER_SIGN);

    }

    public void setSlidePower(double power) {
        slideMotor.set(power);

    }

    // return arm to home position (ground)
    public void returnToHome() {
        // setPower(0);  // will fall to zero due to gravi
        goToPosition(0);
        m_mode = arm_mode.RETURNING_TO_HOME;
    }

    // set arm level 0, 1, 2, 3
    public void goToLevel(int level) {

        switch (level) {
            case 0:
                if (m_mode == arm_mode.RETURNING_TO_HOME) {
                    setElbowPower(0);
                    iSteady = 0;
                    m_mode = arm_mode.SETTLING_HOME;
                } else
                    returnToHome();
                break;

            case 1:
                goToPosition((int)ArmConstants.POSITION1);
                break;

            case 2:
                goToPosition((int)ArmConstants.POSITION2);
                break;

            case 3:
                goToPosition((int)ArmConstants.POSITION3);
                break;
        }

    }

    /**
     * Allow the current arm target position to be adjusted
     */
    public void nudgePosition(int amount) {
        if (m_mode == arm_mode.POSITIONING) {
            m_iTargetPosition += amount;
            if (m_iTargetPosition < 0) m_iTargetPosition = 0;
            goToPosition(m_iTargetPosition);
        }
    }
}
