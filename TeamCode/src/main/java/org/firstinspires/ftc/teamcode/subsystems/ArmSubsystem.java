package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.AngleController;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.SlideController;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.WristController;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

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

    private Servo grabServo;

    private RobotContainer robot;

    private WristController wristController;
    private SlideController slideController;
    private AngleController angleController;

    public ArmSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        this.robot = robot;

        angleController = new AngleController(hardwareMap);
        wristController = new WristController(hardwareMap);
        slideController = new SlideController(hardwareMap);

        grabServo = hardwareMap.get(Servo.class, "grab");

        setWristPositionLevel(0);

    }

    /**
     * Set the subsystem into positioning mode (so it will use the controllers to try and
     * keep the motor holding a certain position)
     *
     * @param position  The position to move arm to (degrees above/below horizontal)
     */
    public void goToPosition(int position) {

        angleController.setDegreesAboveHorizontal( position );
    }

    // Get each control mechanism to do its sensors reads and plant updates
    // whilst relaying metrics between them
    @Override
    public void periodic() {
        slideController.update();
        // Tell the angle controller what the slide extension is as
        angleController.setExtension( slideController.currentPosition() / SlideConstants.SLIDE_MAX );
        angleController.update();
        // Tell wrist the angle of the main arm to complete its gravity calculation
        wristController.setOffsetDegrees( angleController.getDegreesAboveHorizontal() );
        wristController.update();

        wristController.debugInfo().forEach( (k,v) -> robot.addTelem(k,v) );
//        slideController.debugInfo().forEach( (k, v) -> robot.addTelem(k,v) );
//        angleController.debugInfo().forEach( (k, v) -> robot.addTelem(k,v) );
    }

    public void stopSlide() {
        slideController.stop();
    }

    // return arm to home position (ground)
    public void returnToHome() {
        angleController.returnToHome();
    }

    // set arm level 0, 1, 2, 3
    public void goToLevel(int level) {

        switch (level) {
            case 0:
                angleController.returnToHome();
                break;

            case 1:
                angleController.setDegreesAboveHorizontal( ArmConstants.POSITION1 );
                break;

            case 2:
                angleController.setDegreesAboveHorizontal( ArmConstants.POSITION2 );
                break;

            case 3:
                angleController.setDegreesAboveHorizontal( ArmConstants.POSITION3 );
                break;
        }

    }

    /**
     * Allow the current arm target position to be adjusted
     */
    public void nudgePosition(double amount) {
        angleController.adjustSetpoint(amount);
    }

    // Run slidecontroller towards home full power
    public void hoist() {
        slideController.hoist();
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
        slideController.setController(position);
    }

    public boolean slideBusy() {
        return slideController.busy();
    }

    public boolean slideIdle() {
        return slideController.idle();
    }

    public boolean slideCloseToPos() {
        return slideController.closeToPos();
    }

    public void nudgeSlidePosition(double delta) {
        slideController.nudgePosition(delta);
    }

    public void homeSlide() {
        slideController.returnToHome();
    }

    public void setWristPositionLevel(int level) {
        if (level == 0) {
            wristController.setRelativeAngle(WristConstants.WRIST_POS0);
        } else if (level == 1) {
            wristController.setRelativeAngle(WristConstants.WRIST_POS1);
        } else if (level == 2) {
            wristController.setRelativeAngle(WristConstants.WRIST_POS2);
        } else if (level == 3) {
            wristController.setRelativeAngle(WristConstants.WRIST_POS3);
        }

    }

    public void nudgeWristPosition(double delta) {
        wristController.nudgePosition(delta);
    }

    public void setWristRelativeAngle(double angle) {
        wristController.setRelativeAngle(angle);
    }

    public void setWristHorizonAngle(double angle) {
        wristController.setAngleAboveHorizontal(angle);
    }
}
