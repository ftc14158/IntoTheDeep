package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PowerSlewRateLimiter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;
    private final BooleanSupplier boost;

    private static PowerSlewRateLimiter limiterX, limiterY, limiterR;

    Telemetry telemetry;

    public MecanumDriveCommand(MecanumDriveSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX, BooleanSupplier boost, Telemetry telemetry) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;
        this.boost = boost;
        this.telemetry = telemetry;

        limiterX = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_X_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_X_SLEWRATE, 0);
        limiterY = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_Y_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_Y_SLEWRATE, 0);
        limiterR = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_R_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_R_SLEWRATE, 0);

        addRequirements(drive);
    }

    private double squareVal(double val) {
        return val * val * Math.signum(val);
    }

    @Override
    public void execute() {

        double y = leftY.getAsDouble();
        double x = leftX.getAsDouble();
        double r = rightX.getAsDouble();

        if ((DriveConstants.JOYSTICK_MAXPOS_Y_SLEWRATE < 10) || (DriveConstants.JOYSTICK_MAXNEG_Y_SLEWRATE < 10))
            y = limiterY.calculate(y);

        if ((DriveConstants.JOYSTICK_MAXPOS_X_SLEWRATE < 10) || (DriveConstants.JOYSTICK_MAXNEG_X_SLEWRATE < 10))
            x = limiterX.calculate(x);

        if ((DriveConstants.JOYSTICK_MAXPOS_R_SLEWRATE < 10) || (DriveConstants.JOYSTICK_MAXNEG_R_SLEWRATE < 10))
            r = limiterR.calculate(r);

        // cube everything to make easier
        y = squareVal(y);
        x = squareVal(x);
        r = squareVal(r);

        if (! (boost.getAsBoolean())) {
            y = y * DriveConstants.JOYSTICK_MAX_SCALE;
            x = x * DriveConstants.JOYSTICK_MAX_SCALE;
            r = r * DriveConstants.JOYSTICK_MAX_SCALE;
        }

//        telemetry.addData("Joy X,Y,R", String.format("(%.3f, %.3f, %.3f°)", x, y, r) );

//        drive.drive(y, r, x);

        drive.drive(y, x, r);
    }

}
