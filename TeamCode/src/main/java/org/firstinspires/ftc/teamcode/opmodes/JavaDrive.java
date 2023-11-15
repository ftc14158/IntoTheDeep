package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ArmControllerCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryBuildCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;

@TeleOp(group="drive", name = "Drive Robot")
public class JavaDrive extends CommandOpMode {

    private RobotContainer m_robot;

    /* X,Y of 0,0 is center of field.
   X increases to the right looking at center when viewed from behind red alliance wall
   Y increase going away from the center, when viewed from behind red alliance wall
   X and Y are in meters
   Heading is 0 degrees is looking along the positive X axis from the center of the field
   Heading increases turning counter clockwise when looking down at the field from above
    */

    @Override
    public void initialize() {
        m_robot = new RobotContainer( true, hardwareMap,
                gamepad1, gamepad2, new Pose2d(-39,-63.5, Math.toRadians(90)));

        m_robot.getDrivetrain().setDefaultCommand(new MecanumDriveCommand(m_robot.getDrivetrain(),
                () -> -m_robot.getGamepad1().getLeftY(), () -> m_robot.getGamepad1().getLeftX(),
                () -> m_robot.getGamepad1().getRightX()));

        m_robot.getArm().setDefaultCommand( new ArmControllerCommand(m_robot.getArm(),
                () -> m_robot.getGamepad2().getLeftY(), () -> m_robot.getGamepad2().getRightY()) );
/*
        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new TrajectoryBuildCommand(
                        m_robot.getDrivetrain(),
                        (tb) -> {
                            Trajectory traj =
                                    tb.splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                            .splineTo(new Vector2d(0,0), Math.toRadians(0))
                                            .splineTo(new Vector2d(40, -36), Math.toRadians(0))
                                    .build();
                            return traj;
                        }
        ));

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new TrajectoryBuildCommand(
                        m_robot.getDrivetrain(),
                        (tb) -> {
                            Trajectory traj =
                                    tb.lineToSplineHeading(new Pose2d(5, 7, Math.toRadians(180)))
                                            .splineTo(new Vector2d(-42,-20), Math.toRadians(-90))
                                            .lineToSplineHeading(new Pose2d(-42, -60, Math.toRadians(90)))
                                            .build();
                            return traj;
                        }
                ));
*/
        // Set arm position command
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(1) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(2) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(3) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(0) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(true) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(false)) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_GROUND) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_CRUISE) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_MAX) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setWristPositionLevel(0) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setWristPositionLevel(1)) );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.A).whenPressed(

                new SequentialCommandGroup(
                new ArmToGroundCommand( m_robot.getArm() ),
        new WaitUntilCommand(  () -> m_robot.getArm().slideIdle() ),
                new RunCommand( () -> m_robot.getArm().grabOpen(), m_robot.getArm())

            )
        )
        ;

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.X).whenPressed(

                new SequentialCommandGroup(
                        new RunCommand( () -> m_robot.getArm().grabClose(), m_robot.getArm()).withTimeout(500),
                new ArmToCruiseCommand( m_robot.getArm() )
                )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.Y).whenPressed(

                new SequentialCommandGroup(
                        new InstantCommand( () -> m_robot.getArm().goToLevel(2) ),
                        new InstantCommand( () -> m_robot.getArm().setWristPositionLevel(0) ),
                        new InstantCommand( () -> m_robot.getArm().setSlidePosition( ArmConstants.SLIDE_MAX) ),
                        new WaitUntilCommand(  () -> m_robot.getArm().slideIdle() ),
                        new RunCommand( () -> m_robot.getArm().grabOpen(), m_robot.getArm()).withTimeout(1000),
                        new ArmToCruiseCommand( m_robot.getArm() )
                )
        );


        FtcDashboard dashboard = FtcDashboard.getInstance();
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

        schedule(new RunCommand(() -> {
            m_robot.getDrivetrain().update();
            telemetry.addData("Heading", m_robot.getDrivetrain().getPoseEstimate().getHeading());
            telemetry.update();
        }));

    }
}