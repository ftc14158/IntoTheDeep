package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.opmodes.AutonConstants;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.BLUE;
import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.RED;
import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.UNKNOWN;

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
public class AutoPlanCommand extends CommandBase {
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

    private TeamPropDetector.AllianceColor forceColor = null;
    private Boolean forceBackstage = null;
    private boolean leftSlot;  // aim for left slot above april tag

    private static double TILE_WIDTH = 23.5;
    private static double TAPE_WIDTH = 15d/16d;

    public AutoPlanCommand(RobotContainer robot, TeamPropDetector propDetector,
                           TeamPropDetector.AllianceColor forceColor,
                           Boolean forceBackstage, boolean leftSlot) {
        this.robot = robot;
        this.propDetector = propDetector;

        this.forceColor = forceColor;
        this.forceBackstage = forceBackstage;
        this.leftSlot = leftSlot;

        // nobody else starts this this command finishes.
        addRequirements(robot.getDrivetrain(), robot.getArm());
    }

    @Override
    public void execute() {
        color = propDetector.allianceColor;

        position = propDetector.randomizationPosition;
        backstage = propDetector.backstageSide;

        positionIndex = (position == TeamPropDetector.RandomizationPosition.LEFT ? 0 : (position == TeamPropDetector.RandomizationPosition.CENTER ? 1 : 2));

        if (forceColor != UNKNOWN) color = forceColor;
        if (forceBackstage != null) backstage = forceBackstage;

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


            // determine the starting pose
            Pose2d startingPose = getStartingPose();
            robot.getDrivetrain().setPoseEstimate(startingPose);


            // Vision has determined color, so schedule commands based on color, position etc

            Vector2d spikeCenter = getSpikeCenter();

            // TWEAK : for center spike, place 2 inches closer to audience to avoid hit on frame
            if ((position == TeamPropDetector.RandomizationPosition.CENTER) && !backstage)
                spikeCenter = new Vector2d( spikeCenter.getX() - 3, spikeCenter.getY() );

            // select required heading for placing pixel..
            int headingDegrees;
            if (!backstage) {
                // use 91 instead of 90 to get rotation the right way to clear the Team Prop
                // from path
                if (spikeCenter.getX() < -26) headingDegrees = (redAlliance ? 270 : 91);
                else headingDegrees = 0;

                } else {
                // all backstage side placements done facing audience.
                headingDegrees = 180;
            }

            // get the robot pose for placing pixel at the required heading
            Pose2d robotInFrontOfSpike = getRobotPoseFromPixel( spikeCenter, headingDegrees);

            RobotContainer.CommandFactory cmd = robot.commandFactory();


            // Build robot to spike drive
            Command robotToSpike;
//            if (blueAlliance && position == TeamPropDetector.RandomizationPosition.RIGHT) {
//                Pose2d extraPose = new Pose2d( startingPose.getX() +2, startingPose.getY()-28, startingPose.getHeading() );
//                robotToSpike = cmd.buildTraj( tb -> tb.lineToSplineHeading(extraPose).lineToSplineHeading( robotInFrontOfSpike ).build() );
//            } else {
                robotToSpike = cmd.buildTraj( tb -> tb.lineToSplineHeading( robotInFrontOfSpike ).build() );
//            }


            // Target to line up to after pixel placed
            Pose2d linedUpForCenterstage = new Pose2d( -48, 11.75 * bluePlus, Math.toRadians(0));

            // Target after passing under stage door
            Pose2d afterStageDoor = new Pose2d( 30, 9 * bluePlus, Math.toRadians(0));

            // Target in front of backdrop
            // Y co-ord of center of backdrop..
            double backdropY = (TILE_WIDTH * 1.5) * bluePlus;
            // adjust for position - AprilTags are 6 inches apart
            backdropY -= 6 * (positionIndex - 1);

            Pose2d facingBackdrop = new Pose2d( 48, backdropY, Math.toRadians(0) );

            // Final position
            Pose2d parkedPose;
            if (backstage) {
                parkedPose = new Pose2d(47, bluePlus * (14 + TILE_WIDTH * 2), Math.toRadians(0));
            } else {
                parkedPose = new Pose2d(47, bluePlus * 14, Math.toRadians(0));
            }

            Command mainDrive;
            // if on backstage side, simpler route
            if (backstage) {
                mainDrive =                        // further travel to in front of backdrop..
                        cmd.buildTraj( tb -> tb.lineToSplineHeading( facingBackdrop )
                                .build() );

            } else {
                mainDrive =                        // further travel to in front of backdrop..
                        cmd.buildTraj( tb -> tb.lineToSplineHeading( linedUpForCenterstage )
                                .lineToSplineHeading( afterStageDoor )
                                .lineToSplineHeading( facingBackdrop )
                                .build() );

            }

            // When lined up if front if April Tag,
            // tag position to camera is X=0, Y=13

            Log.w("AUTOVISION", "Start pose " +  startingPose.toString());
            Log.w("AUTOVISION", "spikeCenter " + spikeCenter.toString());
            Log.w("AUTOVISION", "robot at spike " + robotInFrontOfSpike.toString());
            Log.w("AUTOVISION", "after " + linedUpForCenterstage.toString());
            Log.w("AUTOVISION", "Required tag " + requiredTag );


            // Schedule commands to run the route..
            CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    cmd.homeSlide(),
                                    cmd.armToGround()
                            ),
                        robotToSpike // cmd.buildTraj( tb -> tb.lineToSplineHeading( robotInFrontOfSpike ).build() )
                    ),
                    // open the grab once arm is finished positioning to ground
                    cmd.openGrab(),
                    cmd.waitMillisecs(500),

                    // try and deposit only one pixel
                    // by retracting about 500 clicks
                    // and grabbing again
                    new ParallelCommandGroup(
                        cmd.raiseOnePixelFromGround(),
                        cmd.buildTraj( tb -> tb.forward(1).build() )
                    ),
                    // open the grab once arm is finished positioning to ground
                    cmd.closeGrab(),
                    // wait for grab to close
                    cmd.waitMillisecs(500),
                    // slightly clear of placed pixel before turning..
                    cmd.buildTraj( tb -> tb.back(  /*blueAlliance && position == TeamPropDetector.RandomizationPosition.RIGHT ? 0.5 :*/ 3).build() ),

                    cmd.waitMillisecs( backstage ? 0 : 3000 ),
                    // retract
                    new ParallelCommandGroup(
                        cmd.armToCruise(),
                        mainDrive
                                /* new SequentialCommandGroup(
                                // further travel to in front of backdrop..
                                cmd.buildTraj( tb -> tb.lineToSplineHeading( linedUpForCenterstage )
                                        .lineToSplineHeading( afterStageDoor )
                                        .lineToSplineHeading( facingBackdrop )
                                        .build() )
                        ) */
                    ),
                    // Align with correct apriltag based on position
                    // Confirm distance from board
                        // Extend arm at correct height and wrist angle
                        // Drop pixel
                        // retract
                        // arm to start pos
                        // park in backstage
                    cmd.armToAprilTagScanPosition(), // armToStartPos(),

                    new AlignToAprilTagCommand(robot, requiredTag, leftSlot).withTimeout((int)AutonConstants.APRILTAG_TIMEOUT),
                    // cmd.buildTraj( tb -> tb.forward(2).build() ),

              //          cmd.buildTraj( tb -> tb.strafeRight(3).build() ),
              //          new WaitUntilCommand( () -> !robot.getDrivetrain().isBusy() ),


                        cmd.raiseOutandRelease(),
                  //      cmd.buildTraj( tb -> tb.back(8).build() ),
                  //  new SequentialCommandGroup(
                  //          cmd.waitMillisecs(00),
                  //          cmd.armToStartPos() ) ,
                    cmd.buildTraj( tb -> tb.lineToSplineHeading( parkedPose ).build() ),
                        new WaitUntilCommand( () -> !robot.getDrivetrain().isBusy() ),
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

        // Add 2 inches off from center if targetting middle spike, to increase clearance from
        // the rigging structure to avoid hitting it during spin
        if (position == TeamPropDetector.RandomizationPosition.CENTER)
            x += 2 * (backstage ? 1 : -1);

        // Determine Y position of center spike
        // Blue is on the plus Y side
        // the Y co-ordinates are a symmetry thrugh the Y origin
        centerY = bluePlus * 24.25;

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
        ry = pixelCenter.getY() + (headingDegrees > 88 && headingDegrees < 92 ? -12 : headingDegrees == 270 ? 12 : 0);
        ry += (headingDegrees == 0 ? -0.5 : headingDegrees == 180 ? 0.5 : 0);
        rx += (headingDegrees > 88 && headingDegrees < 92 ? 0.5 : headingDegrees == 270 ? -0.5 : 0);
        return new Pose2d(rx, ry, Math.toRadians(headingDegrees));
    }
}
