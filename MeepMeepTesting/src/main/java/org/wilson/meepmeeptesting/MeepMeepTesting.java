package org.wilson.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.DriveTrainType;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.util.Vector;

public class MeepMeepTesting {

    static private Pose2d getStartingPose(boolean blue, boolean backstageSide) {
        // X position in either side of origin, depending on if on backstage side
        double x = -11.75 + ((backstageSide ? 1 : -1) * 27.25);

        // Y position depends on if red or blue
        double y = (blue ? 1 : -1) * 62.125;

        // heading depends on if red or blue
        double heading = Math.toRadians( blue ? 270 : 90 );

        return new Pose2d(x, y, heading);
    }

    static private Vector2d getSpikeCenter(boolean red, boolean backstageSide, int position ) {
        double centerX, centerY, x, y;


        // determine the X of the central spike mark.
        // the center point between the two sets of spike marks
        // is at X = -11.75
        centerX = -11.75 + 23.5 * (backstageSide ? 1 : -1);

        // tape is 15/16 inch wide, tile is 22.75 inch between tabs
        // determine the X of the required position
        // Invert the position if looking from the blue side
        x = centerX + ((22.75 - (15d/16d)) / 2d) * ( (red ? position : (2-position) ) - 1);

        // Determine Y position of center spike
        // Blue is on the plus Y side
        // the Y co-ordinates are a symmetry thrugh the Y origin
        centerY = (red ? -1 : 1) * 24.125;

        // The center Y of the side strips is 6 inches minus half a tape width
        // offset from the Y of the central strip
        y = centerY;
        if (position != 1) y += (red ? -1 : 1) * (6. - 15./32);

        return new Vector2d(x,y);
    }

    public static void main(String[] args) {
        MeepMeep meepMeep;
        meepMeep = new MeepMeep(800);

        boolean red = true;
        boolean backstageSide = false;
        int position = 2;
        Vector2d spikeCenter;

        //Pose2d startingPose = getStartingPose(!red, backstageSide);

        Pose2d startingPose = new Pose2d(-39.0, -62.125, Math.toRadians( 90 ) );

        // red audience side start, place on left pixel
        // centerstage exit
        spikeCenter = getSpikeCenter(red, backstageSide, position);
        // required heading..
        int headingDegrees;
        if (!backstageSide) {
            if (spikeCenter.getX() < -26) headingDegrees = (red ? 270 : 90);
            else headingDegrees = 0;
        } else {
            headingDegrees = 180;
        }

        // Adjust target pose by robot offset.
        double rx, ry;
        rx = spikeCenter.getX() + (headingDegrees == 0 ? -12 : headingDegrees == 180 ? 12 :0 );
        ry = spikeCenter.getY() + (headingDegrees == 90 ? -12 : headingDegrees == 270 ? 12 : 0 );
        ry += (headingDegrees == 0 ? -0.5 : headingDegrees == 180 ? 0.5 :0 );
        rx += (headingDegrees == 90 ? 0.5 : headingDegrees == 270 ? -0.5 : 0 );
        Pose2d robotInFrontOfSpike = new Pose2d(rx, ry, Math.toRadians(headingDegrees));

        // Target to line up to after pixel placed
        Pose2d linedUpForCenterstage = new Pose2d( -40, 11.75 * (red ? -1 : 1), Math.toRadians(0));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(17.2, 16.75)
                .setDriveTrainType(DriveTrainType.MECANUM)
                // Set bot constraints to match 2023 bot
                .setConstraints(35, 25, Math.toRadians(150), Math.toRadians(100), 15.73)


                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder( startingPose )
                                // .lineToSplineHeading( robotInFrontOfSpike )
                                .strafeTo( new Vector2d( -49, -35 ))
                                .waitSeconds(2)
                                .setReversed(true)
                                .splineTo( new Vector2d( -57, -57) , Math.toRadians(45+180) )
                                .waitSeconds(2)
                                .setReversed(false)

//                                .lineTo( new Vector2d(12, 10.5 * (red ? -1 : 1) ) )
 //                               .lineToConstantHeading( new Vector2d(48, (red ? -1 : 0) * 36) )
                                .build()
                                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}