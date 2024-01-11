package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.opmodes.AutonConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

import java.util.List;
import java.util.Optional;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;

public class AlignToAprilTagCommand extends CommandBase {
    private final RobotContainer robot;
    private final int tagID;
    private AprilTagProcessor aprilTag = null;
    private boolean finished;
    private double visionStartTimeStamp = 0;
    private boolean aprilTagWasEnabled = false;
    private boolean leftSlot;

    private int phase = 0;

    // forward till FTC Y = 12.2
    // then for left tag, strafe to bearing -9.5, right to bearing 6
    public AlignToAprilTagCommand(RobotContainer robot, int tagID, boolean leftSlot) {
        this.robot = robot;
        this.tagID = tagID;
        this.leftSlot = leftSlot;

        addRequirements(robot.getDrivetrain());
        finished = false;
        phase = 0;
    }

    @Override
    public void execute() {
        checkVision();
        robot.getDrivetrain().update();

        if (phase == 2) {
            finished = !robot.getDrivetrain().isBusy();
            Log.w("APRILTAG", "Running traj. finished = " +finished );
        }
        else {
            finished = false;

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // No tags visible after 3 seconds,  finish
            if (currentDetections.size() == 0) {
                Log.w("APRILTAG", "No detections");
                robot.getDrivetrain().stop();
                // if ( secondsWithoutDetection() > 3.0 ) {
                //     finished = true;
                // }
            } else {

                // Is the required tag visible?
                Optional<AprilTagDetection> requiredTag =
                        currentDetections.stream().filter(d -> d.id == tagID).findFirst();
                if (requiredTag.isPresent()) {

                    // reset vision timestamp for first part
                    if (phase == 0) {
                        setVisionTimeStamp();
                        phase = 1;
                    }

                    // The required tag is visible..
                    // Drive to get closer to Y=13, X=0
                    AprilTagDetection tag = requiredTag.get();
                    // calculate drive error
                    double errorX, errorY;
                    if (leftSlot) {
                        //                    errorX = tag.ftcPose.x - AutonConstants.APRILTAG_SETPOINT_X_LEFT;
                        errorX = AutonConstants.APRILTAG_SETPOINT_B_LEFT - tag.ftcPose.bearing;
                        errorY = AutonConstants.APRILTAG_SETPOINT_Y_LEFT - tag.ftcPose.y;
                    } else {
                        //                    errorX = tag.ftcPose.x - AutonConstants.APRILTAG_SETPOINT_X_RIGHT;
                        errorX = AutonConstants.APRILTAG_SETPOINT_B_RIGHT - tag.ftcPose.bearing;
                        errorY = AutonConstants.APRILTAG_SETPOINT_Y_RIGHT - tag.ftcPose.y;
                    }


                    double powerX = clamp(errorX * AutonConstants.APRILTAG_KX, -AutonConstants.APRILTAG_CLAMP, AutonConstants.APRILTAG_CLAMP);
                    double powerY = clamp(errorY * AutonConstants.APRILTAG_KY, -AutonConstants.APRILTAG_CLAMP, AutonConstants.APRILTAG_CLAMP);

                    Log.w("APRILTAG", "Phase=" + phase + " LeftSlot " + leftSlot + " Error " + errorX + "," + errorY + " Power X/Y " + powerX + "," + powerY);

                    if (phase == 1) {
                        // Adjust only Y till close
                        if ((Math.abs(errorY) < 0.3) || (secondsElapsed() > 1.5))  {
                            phase = 2;
                        } else {
                            powerX = 0;
                            robot.getDrivetrain().drive(powerY, powerX, 0);
                        }
                    }
                    if (phase == 2) {
                        // Run a strafe trajectory based on bearing.
                        robot.getDrivetrain().drive(0, 0, 0);

                        // 1 inch left per 2.5 X is greater than bearing target

                        TrajectorySequence seq = robot.getDrivetrain().trajectorySequenceBuilder( robot.getDrivetrain().getPoseEstimate() )
                                        .strafeRight(errorX / 4.0 ).build();
                        Log.w("APRILTAG", "Phase=" + phase + " Build traj " + (errorX / 4.0) + " inches. Y=" + tag.ftcPose.y + ", B=" + tag.ftcPose.bearing );

                        robot.getDrivetrain().followTrajectorySequence( seq );

                        // just adjust X once Y is close
                    }


                } else {
                    // If adjacent visible.. move 6 inches

                    Optional<AprilTagDetection> adjacentTag =
                            currentDetections.stream().filter(d -> Math.abs(d.id - tagID) < 2).findFirst();

                    if (adjacentTag.isPresent()) {
                        AprilTagDetection adjacent = adjacentTag.get();
                        // Move left is adjacent is higher, right if lower
                        Log.w("APRILTAG", "Adjacent " + (tagID - adjacent.id));

                        //                    robot.getDrivetrain().drive(0, .3 * (tagID - adjacent.id), 0);

                    } else {
                        Log.w("APRILTAG", "No Adjacent");
                        robot.getDrivetrain().stop();
                    }

                }
            }
        }
    }

    @Override
    public boolean isFinished() {

        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        Log.w("APRILTAG","End (Interrupt="+interrupted+")" );
        if (!aprilTagWasEnabled) robot.stopVisionProcessor(aprilTag);
        phase = 0;
        robot.getDrivetrain().stop();
    }

    /**
     * Check robot vision is running apriltag, or start it if not
     */

    private void checkVision() {
        if (aprilTag == null) {
            aprilTag = robot.aprilTagProcessor();
            aprilTagWasEnabled = robot.isVisionProcessorRunning(aprilTag);
            if (!aprilTagWasEnabled) robot.startVisionProcessor(aprilTag);
        }
    }

    private void setVisionTimeStamp() {
        visionStartTimeStamp = (double) System.nanoTime() / 1E9;
    }

    private double secondsElapsed() {
        return ((double) System.nanoTime() / 1E9) - visionStartTimeStamp;
    }
}
