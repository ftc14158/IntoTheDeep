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
public class AutoTestCommand extends CommandBase {
    private TeamPropDetector propDetector;
    private RobotContainer robot;
    private TeamPropDetector.AllianceColor color = TeamPropDetector.AllianceColor.UNKNOWN;
    private TeamPropDetector.RandomizationPosition position;
    private boolean backstage;
    private boolean redAlliance = false;
    private boolean blueAlliance = false;
    private int positionIndex;  // 0 = left, 1 = center, 2 = right
    private int redPlus;  // 1 if Red, -1 if Blue
    private int bluePlus; // 1 if Blue, -1 if Red
    private int requiredTag = 0;

    private static double TILE_WIDTH = 23.5;
    private static double TAPE_WIDTH = 15d/16d;

    public AutoTestCommand(RobotContainer robot, TeamPropDetector propDetector) {
        this.robot = robot;
        this.propDetector = propDetector;

        // nobody else starts this this command finishes.
        addRequirements(robot.getDrivetrain(), robot.getArm());
    }

    @Override
    public void execute() {
        color = RED;
        position = TeamPropDetector.RandomizationPosition.CENTER;
        backstage = false;
        positionIndex = (position == TeamPropDetector.RandomizationPosition.LEFT ? 0 : (position == TeamPropDetector.RandomizationPosition.CENTER ? 1 : 2));

        if (color != TeamPropDetector.AllianceColor.UNKNOWN) {

            // Vision pipeline done so turn off
            robot.stopVisionProcessor( propDetector );

            // Convenience variables for readability
            redAlliance = (color == RED);
            blueAlliance = !redAlliance;
            redPlus = redAlliance ? 1 : -1;
            bluePlus = blueAlliance ? 1 : -1;

            // April Tag ID to align with on backdrop
            requiredTag = 1 + positionIndex + (redAlliance ? 3 : 0);

            // SET STARTING POSE
            Pose2d startingPose = new Pose2d( (TILE_WIDTH * 1.5), (TILE_WIDTH * 1.5) * bluePlus, Math.toRadians(0) );
            robot.getDrivetrain().setPoseEstimate(startingPose);

            // Target in front of backdrop
            // Y co-ord of center of backdrop..
            double backdropY = (TILE_WIDTH * 1.5) * bluePlus;
            // adjust for position - AprilTags are 6 inches apart
            backdropY -= 6 * (positionIndex - 1);

            Pose2d facingBackdrop = new Pose2d( 47, backdropY, Math.toRadians(0) );

            // When lined up if front if April Tag,
            // tag position to camera is X=0, Y=13

            Log.w("AUTOVISION", "Start pose " +  startingPose.toString());
            Log.w("AUTOVISION", "Backdrop " +  facingBackdrop.toString());
            Log.w("AUTOVISION", "Required tag " + requiredTag );

            RobotContainer.CommandFactory cmd = robot.commandFactory();

            // Schedule commands to run the route..
            CommandScheduler.getInstance().schedule(
                    new SequentialCommandGroup(

                            new ParallelCommandGroup(
                                    cmd.armToCruise(),
                                    new SequentialCommandGroup(
                                            // further travel to in front of backdrop..
                                            cmd.buildTraj( tb -> tb.lineToSplineHeading( facingBackdrop )
                                                    .build() )
                                    )
                            ),
                            // Align with correct apriltag based on position
                            // Confirm distance from board
                            // Extend arm at correct height and wrist angle
                            // Drop pixel
                            // retract
                            // arm to start pos
                            // park in backstage
                            cmd.armToStartPos(),
                            new AlignToAprilTagCommand(robot, requiredTag).withTimeout(4000),
                            cmd.armToCruise(),
                            //new RunCommand( () -> robot.getDrivetrain().drive( -.3, 0, 0 ), robot.getDrivetrain()).withTimeout(300),
                            //new InstantCommand( () -> robot.getDrivetrain().stop(), robot.getDrivetrain()),
 //                         //new RunCommand( () -> robot.getDrivetrain().drive( -.3, 0, 0 ), robot.getDrivetrain()).withTimeout(600)
                            cmd.buildTraj( tb -> tb.forward(3).build() ),
                            cmd.raiseOutandRelease(),
                            cmd.buildTraj( tb -> tb.back(3).build() ),
                            cmd.armToStartPos()

                    )

            );

        }
    }

    @Override
    public boolean isFinished() {
        // only finished when vision was determined
        return color != TeamPropDetector.AllianceColor.UNKNOWN;
    }

    private Pose2d getStartingPose() {
        // X position in either side of origin, depending on if on backstage side
        double x = -11.75 + ((backstage ? 1 : -1) * 26.25);

        // Y position depends on if red or blue
        double y = bluePlus * 62.125;

        // heading depends on if red or blue
        double heading = Math.toRadians( color == BLUE ? 270 : 90 );

        return new Pose2d(x, y, heading);
    }

    /** Calculate the center of the spike on which pixel must be placed
     *  based on required detection
     *
     * @return Vector2d of the x,y of the center
     */
    private Vector2d getSpikeCenter() {
        double centerX, centerY, x, y;


        // determine the X of the central spike mark.
        // the center point between the two sets of spike marks
        // is at X = -11.75
        centerX = -11.75 + 23.5 * (backstage ? 1 : -1);

        // tape is 15/16 inch wide, tile is 22.75 inch between tabs
        // determine the X of the required position
        // Invert the position if looking from the blue side
        x = centerX + ((22.75 - (15d/16d)) / 2d) * ( ( redAlliance ? positionIndex : (2-positionIndex) ) - 1);

        // Determine Y position of center spike
        // Blue is on the plus Y side
        // the Y co-ordinates are a symmetry thrugh the Y origin
        centerY = bluePlus * 24.125;

        // The center Y of the side strips is 6 inches minus half a tape width
        // offset from the Y of the central strip
        y = centerY;
        if (position != TeamPropDetector.RandomizationPosition.CENTER) y += bluePlus * (6 - 15/32);

        return new Vector2d(x,y);
    }

    // Calculate where post of robot must be at given heading so that the
    // arm grabber will place pixel at the given position.

    private Pose2d getRobotPoseFromPixel(Vector2d pixelCenter, int headingDegrees) {
        double rx, ry;
        rx = pixelCenter.getX() + (headingDegrees == 0 ? -12 : headingDegrees == 180 ? 12 : 0);
        ry = pixelCenter.getY() + (headingDegrees == 90 ? -12 : headingDegrees == 270 ? 12 : 0);
        ry += (headingDegrees == 0 ? -0.5 : headingDegrees == 180 ? 0.5 : 0);
        rx += (headingDegrees == 90 ? 0.5 : headingDegrees == 270 ? -0.5 : 0);
        return new Pose2d(rx, ry, Math.toRadians(headingDegrees));
    }
}
