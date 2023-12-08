package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ArmStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.*;

@TeleOp(group="drive", name = "Drive Robot")
public class DriveRobot extends CommandOpMode {

    private RobotContainer robot;

    /* X,Y of 0,0 is center of field.
   X increases to the right looking at center when viewed from behind red alliance wall
   Y increase going away from the center, when viewed from behind red alliance wall
   X and Y are in meters
   Heading is 0 degrees is looking along the positive X axis from the center of the field
   Heading increases turning counter clockwise when looking down at the field from above
    */

    @Override
    public void initialize() {
        // Start
        robot = new RobotContainer( this );

        // Set arm position command
        robot.getGamepad2().getGamepadButton(Button.X).whenPressed(
                new InstantCommand( () -> robot.getArm().goToLevel(1) ) );
        robot.getGamepad2().getGamepadButton(Button.Y).whenPressed(
                new InstantCommand( () -> robot.getArm().goToLevel(2) ) );
        robot.getGamepad2().getGamepadButton(Button.BACK).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand( () -> robot.getArm().goToLevel(3) ),
                        new InstantCommand( () -> robot.getArm().setSlidePosition( SlideConstants.SLIDE_MAX )))
        );

        robot.getGamepad1().getGamepadButton(Button.BACK).whileHeld(
                        new InstantCommand( () -> robot.getArm().hoist() )
        ).whenReleased( new InstantCommand( () -> robot.getArm().stopSlide() ));

        robot.getGamepad2().getGamepadButton(Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand( () -> robot.getArm().setWristPositionLevel(0) ),
                        new InstantCommand( () -> robot.getArm().goToLevel(0) ) ) );

        robot.getGamepad2().getGamepadButton(Button.LEFT_BUMPER).whenPressed(
                new InstantCommand( () -> robot.getArm().setGrab(true) ) );
        robot.getGamepad2().getGamepadButton(Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand( () -> robot.getArm().setGrab(false)) );


        robot.getGamepad2().getGamepadButton(Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> robot.getArm().nudgePosition(-ArmConstants.NUDGE_DEGREES) ) );
        robot.getGamepad2().getGamepadButton(Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> robot.getArm().nudgePosition(ArmConstants.NUDGE_DEGREES) ) );
        robot.getGamepad2().getGamepadButton(Button.DPAD_LEFT).whenPressed(
                new InstantCommand( () -> robot.getArm().nudgeSlidePosition(-SlideConstants.SLIDE_PIXEL_HEIGHT) ) );
        robot.getGamepad2().getGamepadButton(Button.DPAD_RIGHT).whenPressed(
                new InstantCommand( () -> robot.getArm().nudgeSlidePosition(SlideConstants.SLIDE_PIXEL_HEIGHT) ) );

        robot.getGamepad2().getGamepadButton(Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand( () -> robot.getArm().setWristPositionLevel(0) ) );
        robot.getGamepad2().getGamepadButton(Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand( () -> robot.getArm().setWristPositionLevel(1)) );

        robot.getGamepad1().getGamepadButton(Button.A).whenPressed(

                new SequentialCommandGroup(
                new ArmToGroundCommand( robot.getArm() ),
        new WaitUntilCommand(  () -> robot.getArm().slideIdle() ),
                new InstantCommand( () -> robot.getArm().grabOpen(), robot.getArm())

            )
        )
        ;

        robot.getGamepad1().getGamepadButton(Button.X).whenPressed(

                new SequentialCommandGroup(
                        new RunCommand( () -> robot.getArm().grabClose(), robot.getArm()).withTimeout(500),
                new ArmToCruiseCommand( robot.getArm() )
                )
        );

        robot.getGamepad1().getGamepadButton(Button.Y).whenPressed(

                new SequentialCommandGroup(
                        new InstantCommand( () -> robot.getArm().goToLevel(2) ),
                        new InstantCommand( () -> robot.getArm().setWristPositionLevel(2) ),
                        new InstantCommand( () -> robot.getArm().setSlidePosition( SlideConstants.SLIDE_MAX) ),
                        new WaitUntilCommand(  () -> robot.getArm().slideIdle() )
                )
        );

        robot.getGamepad1().getGamepadButton(Button.B).whenPressed(

                new ArmStartPositionCommand(  robot.getArm() )
        );

        robot.getGamepad1().getGamepadButton(Button.START).whenPressed(

                new LaunchDroneCommand(robot.getDroneSubsystem()).withTimeout(3000)
        );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        if (dashboard != null) schedule( new PerpetualCommand( new RunCommand( () -> { robot.sendTelem( dashboard );})));


        robot.getDrivetrain().setDefaultCommand(new MecanumDriveCommand(robot.getDrivetrain(),
                () -> -robot.getGamepad1().getLeftY(), () -> robot.getGamepad1().getLeftX(),
                () -> robot.getGamepad1().getRightX(), () -> robot.getGamepad1().getGamepadButton(Button.LEFT_BUMPER).get()));

        schedule(new RunCommand(() -> {
            robot.getDrivetrain().update();
            telemetry.addData("Heading", Math.toDegrees(robot.getDrivetrain().getPoseEstimate().getHeading()) );
            telemetry.update();
        }));

    }
}