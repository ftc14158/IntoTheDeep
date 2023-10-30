package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.Function;
import java.util.function.Supplier;

public class TrajectoryBuildCommand extends CommandBase {

    private final MecanumDriveSubsystem drive;
    private final Function<TrajectoryBuilder, Trajectory> trajectorySupplier;

    public TrajectoryBuildCommand(MecanumDriveSubsystem drive, Function<TrajectoryBuilder, Trajectory> trajectorySupplier) {
        this.drive = drive;
        this.trajectorySupplier = trajectorySupplier;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.followTrajectory( trajectorySupplier.apply( drive.trajectoryBuilder( drive.getPoseEstimate() ) ) );
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}
