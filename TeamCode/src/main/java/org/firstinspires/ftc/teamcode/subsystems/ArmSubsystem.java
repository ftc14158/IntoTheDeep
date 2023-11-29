package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    // private MotorEx elbowMotor2;

    private MotorEx slideMotor;
    private arm_mode slideMode;

    // potVoltage top 0.833v
    // bottom 2.55v,
    private AnalogInput elbowAngle;

    private MotorEx wristMotor;

    private Servo grabServo;

    private RobotContainer m_robot;

    private arm_mode m_mode;
    private arm_mode wristMode;

    private int m_iTargetPosition = 0;

    private ArmFeedforward m_positionController;

    // F term of no use in arm as it multiples by the set point, which is not a useful feedforward value
    // so ArmFeedForward used instead for the feed forward calc
    private PIDController m_PController;

    private PIDController m_slidePIDController;

    private int lastPosition = 0;
    private int lastSlidePosition = 0;

    private int lastWristPosition = 0;
    private int lastWristSteady = 0;
    private int wristHomePosition = 0;

    private String lastMsg = "";
    private int iSteady;
    private double lastTimeStamp;

    // Note: wrist motor negative power / negative count is UPWARD

    public ArmSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        m_robot = robot;

        m_PController = new PController(1);
        m_positionController = new ArmFeedforward( .1, .1, 0 );
        m_slidePIDController = new PIDController(ArmConstants.SLIDE_KP, ArmConstants.SLIDE_KI, 0);

        elbowMotor1 = new MotorEx(hardwareMap, ArmConstants.ELBOW_MOTOR1);
        // elbowMotor2 = new MotorEx(hardwareMap, ArmConstants.ELBOW_MOTOR2);
        slideMotor = new MotorEx(hardwareMap, ArmConstants.SLIDE_MOTOR);
        wristMotor = new MotorEx(hardwareMap, ArmConstants.WRIST_MOTOR);

        elbowAngle = hardwareMap.analogInput.get("elbowAngle");
        grabServo = hardwareMap.get(Servo.class, "grab");

        elbowMotor1.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
//        elbowMotor1.setInverted(true);
        elbowMotor1.resetEncoder();
        elbowMotor1.setRunMode(Motor.RunMode.RawPower);
        elbowMotor1.setDistancePerPulse(1);
        elbowMotor1.setPositionCoefficient( 0 ); // 0.02 );

/*
//        elbowMotor2.setInverted(true);
        elbowMotor2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        elbowMotor2.resetEncoder();
        elbowMotor2.setRunMode(Motor.RunMode.RawPower);
        elbowMotor2.setDistancePerPulse(1);
        elbowMotor2.setPositionCoefficient( 0 ); // .02 );
*/
        slideMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        slideMotor.setInverted(true);
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        slideMotor.set(0);
        slideMode = arm_mode.IDLE;
        slideMotor.resetEncoder();

        m_mode = arm_mode.IDLE;
        iSteady = 0;

        wristMotor.stopMotor();
        wristMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        wristMotor.setRunMode(Motor.RunMode.RawPower);
        wristMode = arm_mode.RETURNING_TO_HOME;


    }


    /**
     * Set the subsystem into positioning mode (so it will use the controllers to try and
     * keep the motor holding a certain position)
     *
     * @param position  The position to move arm to (in encoder ticks)
     */
    public void goToPosition(int position) {


        lastTimeStamp = 0;
        m_PController.setPID(ArmConstants.ARM_KP, ArmConstants.ARM_KI, ArmConstants.ARM_KD );
        m_positionController = new ArmFeedforward(ArmConstants.ARM_KS, ArmConstants.ARM_KCOS, ArmConstants.ARM_KV, ArmConstants.ARM_KA);

        //        Log.w("ARM", "Going to position " + position);
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
        int currentPosition = (int)(((2.55 - elbowAngle.getVoltage()) * 500.)); //   elbowMotor1.getCurrentPosition();

        int currentSlidePosition = slideMotor.getCurrentPosition();
        int slidePositionDiff = currentSlidePosition - lastSlidePosition;
        lastSlidePosition = currentSlidePosition;

        int currentWristPosition = wristMotor.getCurrentPosition();

        m_robot.addTelem("Pot voltage", elbowAngle.getVoltage());

        m_robot.addTelem("Slide pos", currentSlidePosition);
        double slideerror = 0;
        double slidepower = 0;

        m_robot.addTelem("Wrist position", wristMotor.getCurrentPosition());
        m_robot.addTelem("Wrist home", wristHomePosition);
        m_robot.addTelem("Wrist power", wristMotor.get());
        m_robot.addTelem("Wrist mode", wristMode);

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
        double lastPositionRadians =  ArmConstants.ARM_HOME_RADS + ArmConstants.ARM_POS_TO_RADS * lastPosition;

        double currentTimeStamp = (double) System.nanoTime() / 1E9;
        if (lastTimeStamp == 0) lastTimeStamp = currentTimeStamp;
        double period = currentTimeStamp - lastTimeStamp;
        lastTimeStamp = currentTimeStamp;

        double vel = 0;
        if (Math.abs(period) > 1E-6) {
            vel = (currentPosition - lastPosition) / period;
        }

        //double vel = elbowMotor1.getVelocity();
        //double accel = elbowMotor1.getAcceleration();
        //double accelRads = accel * ArmConstants.ARM_POS_TO_RADS;
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
                // clamp p_error to a max value
                p_error = Math.min( Math.max(p_error, -ArmConstants.ARM_PCLAMP), ArmConstants.ARM_PCLAMP);


                double ff_error =  m_positionController.calculate(currentPositionRadians, velRads);
                // increase kcos error based on slide length
                ff_error += ( (ArmConstants.ARM_KCOS_EXT * currentSlidePosition) * Math.cos(currentPositionRadians) );

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
      //                  elbowMotor2.resetEncoder();
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

        switch(slideMode) {
            case POSITIONING:
                // if at position
                if ( (slidePositionDiff < 2) && Math.abs( currentSlidePosition - m_slidePIDController.getSetPoint() ) < ArmConstants.SLIDE_TOLERANCE ) {
                    // slideMotor.atTargetPosition()) {
                    slideMotor.stopMotor();
                    slideMode = arm_mode.IDLE;
                } else {
                    slideerror = m_slidePIDController.calculate( currentSlidePosition );
                    slidepower = slideerror * ArmConstants.SLIDE_POWER_SIGN;
                    slideMotor.set(slidepower);
                }
                break;
            case IDLE:
                slideerror =0;
                slidepower = 0;
                slideMotor.stopMotor();
                slideMotor.setRunMode(Motor.RunMode.RawPower);

        }
//        m_robot.addTelem("Elbow Power", elbowMotor1.get());

        // Wrist motor control
        switch(wristMode) {
            case RETURNING_TO_HOME:
                wristMotor.set(-ArmConstants.WRIST_DRIVE_SIGN);  // keep motor on upwards
                if (currentWristPosition != lastWristPosition) {
                    lastWristSteady = 0;
                } else {
                    lastWristSteady++;
                    if (lastWristSteady > 10) {
                        // we have found the uppermost position
                        wristHomePosition = currentWristPosition;
                        wristMode = arm_mode.POSITIONING;
                        wristMotor.setRunMode(Motor.RunMode.PositionControl);
                        wristMotor.setTargetPosition( wristHomePosition + (int)ArmConstants.WRIST_POS0 );
                    }
                }

            case POSITIONING:
                wristMotor.setPositionCoefficient(ArmConstants.WRIST_KP);
                wristMotor.set(ArmConstants.WRIST_DRIVE_SIGN);
                break;

            case IDLE:
                wristMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
                wristMotor.set(0);

        }

        m_robot.addTelem("Slide setpoint", m_slidePIDController.getSetPoint());
        m_robot.addTelem("Slide error", slideerror);
        m_robot.addTelem("Slide power", slidepower);
        lastPosition = currentPosition;
        lastWristPosition = currentWristPosition;
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
        // elbowMotor2.set(power * ArmConstants.ARM_POWER_SIGN);

    }

    public void stopSlide() {
        slideMode = arm_mode.IDLE;
        slideMotor.stopMotor();
    }

    public void setSlidePower(double power) {
        if (slideMode == arm_mode.IDLE) slideMotor.set(power);
//        m_robot.addTelem("Slide power", power);

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

    private void setGrabServo(double angle) {
        grabServo.setPosition(angle / 180.);
    }

    public void setGrab(boolean open) {
        setGrabServo( open ? ArmConstants.GRAB_OPEN : ArmConstants.GRAB_CLOSED);
    }

    public void grabOpen() {
        setGrab(true);
    }

    public void grabClose() {
        setGrab(false);
    }

    public void setSlidePosition(double position) {
        Log.i("ARM SLIDE", "Pos " + position);
        // slideMotor.stopMotor();
        m_slidePIDController.setPID(ArmConstants.SLIDE_KP, ArmConstants.SLIDE_KI, ArmConstants.SLIDE_KD);
        m_slidePIDController.setSetPoint(position);
        slideMotor.setRunMode(Motor.RunMode.RawPower);
        //slideMotor.setPositionTolerance(ArmConstants.SLIDE_TOLERANCE);
        //slideMotor.setPositionCoefficient(ArmConstants.SLIDE_KP);
       // slideMotor.
       // slideMotor.setTargetPosition((int)position);
        slideMode = arm_mode.POSITIONING;
    }

    public boolean slideBusy() {
        return slideMode != arm_mode.IDLE;
    }

    public boolean slideIdle() {
        return !slideBusy();
    }

    public boolean slideCloseToPos() {
        return (slideIdle()) || (( Math.abs( m_slidePIDController.getSetPoint() - lastSlidePosition)) < 100 );
    }
    public void nudgeSlidePosition(double delta) {
        setSlidePosition( slideMotor.getCurrentPosition() + delta );
    }

    public void setWristPower(double power) {
        wristMotor.set(power);
    }

    public void setWristPositionLevel(int level) {
        if (level == 0) {
            wristMotor.setTargetPosition(wristHomePosition + ((int)ArmConstants.WRIST_POS0));
        } else if (level == 1) {
            wristMotor.setTargetPosition(wristHomePosition + ((int)ArmConstants.WRIST_POS1));
        }
    }
}
