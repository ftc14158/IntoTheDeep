package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.util.PowerSlewRateLimiter;

import java.util.function.DoubleSupplier;

public class MecanumDriveCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final DoubleSupplier leftY, leftX, rightX;

    private static PowerSlewRateLimiter limiterX, limiterY, limiterR;

    public MecanumDriveCommand(MecanumDriveSubsystem drive, DoubleSupplier leftY,
                               DoubleSupplier leftX, DoubleSupplier rightX) {
        this.drive = drive;
        this.leftX = leftX;
        this.leftY = leftY;
        this.rightX = rightX;

        limiterX = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_X_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_X_SLEWRATE, 0);
        limiterY = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_Y_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_Y_SLEWRATE, 0);
        limiterR = new PowerSlewRateLimiter(DriveConstants.JOYSTICK_MAXPOS_R_SLEWRATE, DriveConstants.JOYSTICK_MAXNEG_R_SLEWRATE, 0);
        addRequirements(drive);
    }

    @Override
    public void execute() {
        // square joystick
        double y = limiterY.calculate(leftY.getAsDouble());
        double x = limiterX.calculate(leftX.getAsDouble());
        double r = limiterR.calculate(rightX.getAsDouble());

        drive.drive(y, x, r);
    }

}
