package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;

public class LaunchDroneCommand extends CommandBase {

    private DroneSubsystem droneSubsystem;

    public LaunchDroneCommand(DroneSubsystem droneSubsystem) {
        this.droneSubsystem = droneSubsystem;

        addRequirements(droneSubsystem);
    }

    @Override
    public void execute() {
        droneSubsystem.launch();
    }

    @Override
    public void end(boolean interrupted) {
        droneSubsystem.stop();
    }
}
