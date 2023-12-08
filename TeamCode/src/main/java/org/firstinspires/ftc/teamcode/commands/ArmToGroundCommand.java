package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;

public class ArmToGroundCommand extends SequentialCommandGroup {
    private ArmSubsystem arm;

    public ArmToGroundCommand(ArmSubsystem arm) {
        this.arm = arm;

        addCommands(
                // ensure grab closed
                new InstantCommand( arm::grabClose ),
                // Make sure slide is retracted first
                new InstantCommand( () -> { arm.setSlidePosition(SlideConstants.SLIDE_CRUISE);}),
                new WaitUntilCommand( arm::slideCloseToPos ),
                // Then move to bottom level and start extending slide
                new InstantCommand( () -> arm.setWristPositionLevel(0)),
                new InstantCommand( () -> { arm.goToLevel( 0 );} ), // .withTimeout(800),
                new InstantCommand( () -> { arm.setSlidePosition(SlideConstants.SLIDE_GROUND);}),
                new WaitUntilCommand( arm::slideCloseToPos )
        );
        addRequirements(arm);
    }



}
