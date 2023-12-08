package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;

public class ArmToCruiseCommand extends SequentialCommandGroup {

    private ArmSubsystem arm;

    public ArmToCruiseCommand(ArmSubsystem arm) {
        this.arm = arm;

        addCommands(
                // Retract slide first
                new InstantCommand( () -> { arm.setSlidePosition(SlideConstants.SLIDE_CRUISE);}),
                new WaitUntilCommand( arm::slideCloseToPos ),
                // Then move to cruise level
                new InstantCommand( () -> { arm.setWristPositionLevel(1);} ),
                new InstantCommand( () -> { arm.goToLevel( 1 );} )
                );
        addRequirements(arm);
    }


}
