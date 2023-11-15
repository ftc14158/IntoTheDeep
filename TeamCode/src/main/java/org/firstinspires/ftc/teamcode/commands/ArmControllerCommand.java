package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.function.DoubleSupplier;

public class ArmControllerCommand extends CommandBase {

        private final ArmSubsystem arm;
        private final DoubleSupplier leftY, rightY;

        public ArmControllerCommand(ArmSubsystem arm,
                                   DoubleSupplier leftY, DoubleSupplier rightY) {
            this.arm = arm;
            this.leftY = leftY;
            this.rightY = rightY;

            addRequirements(arm);
        }

        @Override
        public void execute() {
            arm.setWristPower(leftY.getAsDouble());
            arm.setSlidePower(rightY.getAsDouble());
        }


}
