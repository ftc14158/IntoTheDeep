package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.RED;

/**
 * Command to wait for vision processor results
 * and then schedule actions
 *
 * Route planning notes:
 *
 * Pixel is placed 12 inches in front of center of robot, and 0.5 inches left of center of robot
 *
 * This plan places pixel and crosses field via center to reach backdrop, while avoiding
 * area around other set of spike marks
 *
 */
public class MeepMeepTestCommand extends CommandBase {
    private TeamPropDetector propDetector;
    private RobotContainer robot;
    private boolean backstage;
    private boolean redAlliance = false;
    private boolean blueAlliance = false;
    private int positionIndex;  // 0 = left, 1 = center, 2 = right
    private int redPlus;  // 1 if Red, -1 if Blue
    private int bluePlus; // 1 if Blue, -1 if Red
    private int requiredTag = 0;

    private static double TILE_WIDTH = 23.5;
    private static double TAPE_WIDTH = 15d/16d;

    public MeepMeepTestCommand(RobotContainer robot) {
        this.robot = robot;

        // nobody else starts this this command finishes.
        addRequirements(robot.getDrivetrain(), robot.getArm());
    }

    @Override
    public void execute() {

        // SET STARTING POSE
        Pose2d startingPose = new Pose2d(-39.0, -62.125, Math.toRadians( 90 ) );

        robot.getDrivetrain().setPoseEstimate(startingPose);

        RobotContainer.CommandFactory cmd = robot.commandFactory();

        // Schedule commands to run the route..
        CommandScheduler.getInstance().schedule(
                cmd.buildTraj( tb ->
                                tb.waitSeconds(1)
                                        // .lineToSplineHeading( robotInFrontOfSpike )
                                        .lineToSplineHeading(new Pose2d(-49, 0, Math.toRadians(90)) )
                                        .forward(20)
                                        .splineToLinearHeading(new Pose2d(-38, 45, Math.toRadians(0)), Math.toRadians(0) )
                                        .forward(75)
                                        .lineToSplineHeading(new Pose2d(56, 0, Math.toRadians(270)))
                                        .forward(30)
                                        .lineToSplineHeading(new Pose2d(36, -45, Math.toRadians(180)))
                                        .lineToSplineHeading(new Pose2d(-39, -55, Math.toRadians(90)))
                                        .build()

                )

        );

    }

}
