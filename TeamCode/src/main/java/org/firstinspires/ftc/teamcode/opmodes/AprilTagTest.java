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
import org.firstinspires.ftc.teamcode.commands.AlignToAprilTagCommand;
import org.firstinspires.ftc.teamcode.commands.ArmControllerCommand;
import org.firstinspires.ftc.teamcode.commands.ArmStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.LaunchDroneCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(group="drive", name = "AprilTag Test")
public class AprilTagTest  extends CommandOpMode {

    private RobotContainer m_robot;
    private AprilTagProcessor aprilTag;

    @Override
    public void initialize() {
        m_robot = new RobotContainer( true, hardwareMap, telemetry,
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
                new SequentialCommandGroup(
                        new InstantCommand( () -> m_robot.getArm().goToLevel(3) ),
                        new InstantCommand( () -> m_robot.getArm().setSlidePosition( ArmConstants.SLIDE_MAX )))
        );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand( () -> m_robot.getArm().setWristPositionLevel(0) ),
                        new InstantCommand( () -> m_robot.getArm().goToLevel(0) ) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(true) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(false)) );


        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgePosition(-30) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgePosition(30) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgeSlidePosition(-100) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgeSlidePosition(100) ) );

        /*
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_GROUND) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_CRUISE) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_MAX) ) );
*/
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
                        new InstantCommand( () -> m_robot.getArm().setWristPositionLevel(2) ),
                        new InstantCommand( () -> m_robot.getArm().setSlidePosition( ArmConstants.SLIDE_MAX) ),
                        new WaitUntilCommand(  () -> m_robot.getArm().slideIdle() )
                        //  new RunCommand( () -> m_robot.getArm().grabOpen(), m_robot.getArm()).withTimeout(1000),
                        //  new ArmToCruiseCommand( m_robot.getArm() )
                )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.B).whenPressed(

                new ArmStartPositionCommand(  m_robot.getArm() )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.START).whenPressed(

                new LaunchDroneCommand(m_robot.getDroneSubsystem()).withTimeout(3000)
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.BACK).whileHeld(
                new AlignToAprilTagCommand( m_robot, 5 )
        );

        FtcDashboard dashboard = FtcDashboard.getInstance();
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

        m_robot.initVision();
        aprilTag = m_robot.aprilTagProcessor();
        m_robot.startVisionProcessor( aprilTag );

        schedule(new RunCommand(() -> {
            m_robot.getDrivetrain().update();
            telemetryAprilTag();
            telemetry.addData("Heading", m_robot.getDrivetrain().getPoseEstimate().getHeading());
            telemetry.update();
        }));

    }

    /**
     * Add telemetry about AprilTag detections.
     */
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }   // end method telemetryAprilTag()


}
