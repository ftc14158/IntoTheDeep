package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.WristController;

@TeleOp(group="test", name = "Test Wrist")
public class WristTest extends CommandOpMode {

    private WristController wristController;

    private GamepadEx pad1;


    @Override
    public void initialize() {

        pad1 = new GamepadEx(gamepad1);
        wristController = new WristController(hardwareMap);

        pad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(ArmConstants.WRIST_POS0 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(ArmConstants.WRIST_POS1 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(ArmConstants.WRIST_POS2 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( wristController::stop ) );

        schedule(new RunCommand(() -> {
            wristController.update();

            telemetry.addLine("Gamepad1: A=stop,X=POS0,Y=POS1,B=POS2");
            wristController.debugInfo().forEach( (k,v) -> telemetry.addData(k,v) );

            telemetry.update();
        }));
    }
}
