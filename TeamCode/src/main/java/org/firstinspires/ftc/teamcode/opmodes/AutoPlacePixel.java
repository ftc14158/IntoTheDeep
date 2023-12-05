package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryBuildCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(name="RED Place Pixel")
public class AutoPlacePixel extends CommandOpMode {
    private RobotContainer m_robot;

    @Override
    public void initialize() {

        // Create an FTC dashboard object to use to send back telemetry information
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Create the main robot object that contains the various
        // subsystems etc
        m_robot = new RobotContainer( true, hardwareMap, telemetry,
                gamepad1, gamepad2, new Pose2d(-39,-62.125, Math.toRadians(90)));

        // Create a command that will run every time in the loop to send back the telemetry to
        // the FTC dashboard
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

                // Schedule the two commands to happen one after the other, with some
        // dead commands for time delays in betweeen

        schedule( new SequentialCommandGroup(
                new ParallelCommandGroup(
                new ArmToGroundCommand(m_robot.getArm()),

                new TrajectoryBuildCommand(
                        m_robot.getDrivetrain(),
                        (tb) -> {
                            TrajectorySequence traj = tb
                                    .lineToSplineHeading(new Pose2d(-35.5 -11.8, -23.5, Math.toRadians(0)) )
                                            .build();
                            return traj;
                        }
                ) ),
                // open the grab once arm is finished positioning to ground
                new InstantCommand( () -> m_robot.getArm().grabOpen(), m_robot.getArm() ),
                // wait for grab to open
                new RunCommand( () -> {} ).withTimeout(1000),
                // try and deposit only one pixel
                // by retracting about 500 clicks
                // and grabbing again
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                            new InstantCommand( () -> { m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_GROUND - ArmConstants.SLIDE_PIXEL_HEIGHT);}),
                            new WaitUntilCommand( () -> m_robot.getArm().slideCloseToPos() )
                        ),
                        new TrajectoryBuildCommand(
                                m_robot.getDrivetrain(),
                                (tb) -> {
                                    TrajectorySequence traj =
                                            tb.forward(1)
                                                    .build();
                                    return traj;
                                }
                        ) ),


                // open the grab once arm is finished positioning to ground
                new InstantCommand( () -> m_robot.getArm().grabClose(), m_robot.getArm() ),
                // wait for grab to open
                new RunCommand( () -> {} ).withTimeout(700),

                // retract
                new ParallelCommandGroup(
                    new ArmToCruiseCommand( m_robot.getArm()),

                        new SequentialCommandGroup(
                        new TrajectoryBuildCommand(
                                m_robot.getDrivetrain(),
                                (tb) -> {
                                    TrajectorySequence traj =
                                            tb.splineToConstantHeading( new Vector2d(-35.5, -10.5), Math.toRadians(0))
                                    //lineToSplineHeading(new Pose2d(-39, -60, Math.toRadians(0)) )
                                    //                .lineTo( new Vector2d(12, -60))
                                            //        .lineToConstantHeading( new Vector2d(48, -36) )
                                                    .build();
                                    return traj;
                                }
                        )
                                /*,
                                new TrajectoryBuildCommand(
                                        m_robot.getDrivetrain(),
                                        (tb) -> {
                                            Trajectory traj =
                                                    tb.lineTo( new Vector2d(12, -60))
                                                            .build();
                                            return traj;
                                        }
                                ),
                                new TrajectoryBuildCommand(
                                        m_robot.getDrivetrain(),
                                        (tb) -> {
                                            Trajectory traj =
                                                    tb.lineToConstantHeading( new Vector2d(48, -36) )
                                                            .build();
                                            return traj;
                                        }
                                )  */

                        )
                ),

                new SequentialCommandGroup(
                        new InstantCommand( () -> { m_robot.getArm().setWristPositionLevel(0);} ),
                        new InstantCommand( () -> { m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_MAX);}),
                        new WaitUntilCommand( () -> m_robot.getArm().slideCloseToPos() )
                ),

                new InstantCommand( () -> m_robot.getArm().grabOpen(), m_robot.getArm() ),

                new RunCommand( () -> {} ).withTimeout(1000),

                new InstantCommand( () -> m_robot.getArm().grabClose(), m_robot.getArm() ),
                new SequentialCommandGroup(
                        new ArmToCruiseCommand( m_robot.getArm() ),
                        // new InstantCommand( () -> { m_robot.getArm().setSlidePosition(ArmConstants.SLIDE_CRUISE);}),
                       // new WaitUntilCommand( () -> m_robot.getArm().slideCloseToPos() )
/*
                        new SequentialCommandGroup(
                                new TrajectoryBuildCommand(
                                        m_robot.getDrivetrain(),
                                        (tb) -> {
                                            Trajectory traj =
                                                    tb.lineToConstantHeading( new Vector2d(12, -60) )
                                                            .build();
                                            return traj;
                                        }
                                ),
                                new TrajectoryBuildCommand(
                                        m_robot.getDrivetrain(),
                                        (tb) -> {
                                            Trajectory traj =
                                                    tb.lineToConstantHeading( new Vector2d(-39, -59))
                                                            .build();
                                            return traj;
                                        }
                                )

                                )
                ),
*/
                //new InstantCommand( () -> m_robot.getDrivetrain().turn( Math.toRadians(-90))) ,
                new InstantCommand( () -> m_robot.getArm().goToLevel(0)),
                new RunCommand( () -> {} ).withTimeout(1000)

                )
                )
        );


    }

}
