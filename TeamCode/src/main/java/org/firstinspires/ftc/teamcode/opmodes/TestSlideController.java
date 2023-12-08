package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.SlideController;

@TeleOp(group="test", name = "Test Slide")
public class TestSlideController extends CommandOpMode  {
    private SlideController slideController;

    private GamepadEx pad1;

    @Override
    public void initialize() {

        pad1 = new GamepadEx(gamepad1);
        slideController = new SlideController(hardwareMap);
        slideController.stop();

        pad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> slideController.setController(SlideConstants.SLIDE_CRUISE ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> slideController.setController(SlideConstants.SLIDE_GROUND ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand( () -> slideController.setController(SlideConstants.SLIDE_MAX ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( slideController::stop) );

        pad1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand( slideController::returnToHome ) );

        schedule(new RunCommand(() -> {
            slideController.update();

            telemetry.addLine("Gamepad1: A=stop,X=CRUISE,Y=GROUND,B=MAX,BACK=Home");

            slideController.debugInfo().forEach( (k,v) -> telemetry.addData(k,v) );
            telemetry.update();
        }));
    }
}
