package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.WristConstants;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.AngleController;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.WristController;

@TeleOp(group="test", name = "Test Wrist")
@Disabled
public class TestWristController extends CommandOpMode {

    private WristController wristController;

    private AngleController angleController;

    private GamepadEx pad1;

    private double armAngle = 0;

    private double grabAngle = 90;
    private Servo grabServo;

    private void setGrabServo(double angle) {
        grabServo.setPosition(angle / 180.);
    }

    @Override
    public void initialize() {

        pad1 = new GamepadEx(gamepad1);
        wristController = new WristController(hardwareMap);
        angleController = new AngleController(hardwareMap);

        grabServo = hardwareMap.get(Servo.class, "grab");


        pad1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(WristConstants.WRIST_POS0 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(WristConstants.WRIST_POS1 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand( () -> wristController.setRelativeAngle(WristConstants.WRIST_POS2 ) ) );

        pad1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand( () -> wristController.setAngleAboveHorizontal(0) ) );

        pad1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( wristController::stop ) );

        pad1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> { armAngle += 10; wristController.setOffsetDegrees(armAngle); } ) );
        pad1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> { armAngle -= 10; wristController.setOffsetDegrees(armAngle); } ) );

        pad1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand( () -> { grabAngle = Math.min(180., grabAngle+10.) ; } ) );

        pad1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand( () -> { grabAngle = Math.max(0., grabAngle-10.) ; } ) );

        schedule(new RunCommand(() -> {
            angleController.update();
            armAngle = angleController.getDegreesAboveHorizontal();

            wristController.setOffsetDegrees( angleController.getDegreesAboveHorizontal() );
            wristController.update();

            setGrabServo( grabAngle );
            telemetry.addLine("Gamepad1: A=stop,X=POS0,Y=POS1,B=POS2,DPAD=Nudge arm angle<hr>");
            wristController.debugInfo().forEach( (k,v) -> telemetry.addData(k,v) );
            telemetry.addData("Arm Angle", armAngle);
            telemetry.addData("Grab Angle", grabAngle);
            telemetry.update();
        }));
    }
}
