package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.opmodes.AutonConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
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

    public AlignToAprilTagCommand(RobotContainer robot, int tagID) {
        this.robot = robot;
        this.tagID = tagID;

        addRequirements(robot.getDrivetrain());
        finished = false;
    }

    @Override
    public void execute() {
        checkVision();
        robot.getDrivetrain().update();

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        // No tags visible after 3 seconds,  finish
        if (currentDetections.size() == 0) {
            Log.w("APRILTAG","No detections");
            robot.getDrivetrain().stop();
           // if ( secondsWithoutDetection() > 3.0 ) {
           //     finished = true;
           // }
        } else {
            // reset vision timestamp
            setVisionTimeStamp();

            // Is the required tag visible?
            Optional<AprilTagDetection> requiredTag =
                    currentDetections.stream().filter(d -> d.id == tagID).findFirst();
            if (requiredTag.isPresent()) {
                // The required tag is visible..
                // Drive to get closer to Y=13, X=0
                AprilTagDetection tag = requiredTag.get();
                // calculate drive error
                double errorX = tag.ftcPose.x - AutonConstants.APRILTAG_SETPOINT_X;
                double errorY = AutonConstants.APRILTAG_SETPOINT_Y - tag.ftcPose.y;
                Log.w("APRILTAG","Required error "+errorX+","+errorY);

                double powerX = clamp(errorX * AutonConstants.APRILTAG_KX, -AutonConstants.APRILTAG_CLAMP, AutonConstants.APRILTAG_CLAMP);
                double powerY = clamp(errorY * AutonConstants.APRILTAG_KX, -AutonConstants.APRILTAG_CLAMP, AutonConstants.APRILTAG_CLAMP);

                robot.getDrivetrain().drive( errorY * AutonConstants.APRILTAG_KY, errorX * AutonConstants.APRILTAG_KX, 0 );

            } else {
                // If adjacent visible.. move 6 inches

                Optional<AprilTagDetection> adjacentTag =
                        currentDetections.stream().filter(d -> Math.abs(d.id - tagID) < 2).findFirst();

                if (adjacentTag.isPresent()) {
                    AprilTagDetection adjacent = adjacentTag.get();
                    // Move left is adjacent is higher, right if lower
                    Log.w("APRILTAG","Adjacent " + (tagID - adjacent.id) );

                    robot.getDrivetrain().drive(0, .3 * (tagID - adjacent.id), 0);

                }
                else {
                    Log.w("APRILTAG","No Adjacent" );
                    robot.getDrivetrain().stop();
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

    private double secondsWithoutDetection() {
        return ((double) System.nanoTime() / 1E9) - visionStartTimeStamp;
    }
}
