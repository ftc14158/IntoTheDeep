package org.firstinspires.ftc.teamcode.opmodes;


import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;
import org.firstinspires.ftc.teamcode.subsystems.WristConstants;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.AngleController;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.SlideController;
import org.firstinspires.ftc.teamcode.subsystems.mechanism.WristController;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.*;

@TeleOp(group="test", name = "Test Arm Angle")
@Disabled
public class TestAngleController extends CommandOpMode {

    private AngleController angleController;
    private SlideController slideController;
    private WristController wristController;

    private GamepadEx pad1;

    private double target = 0;

    private RobotContainer robot;

    private void btnPressCmd(Button btn, Command cmd) {
        pad1.getGamepadButton(btn).whenPressed(cmd);
    }

    @Override
    public void initialize() {

     //   robot = new RobotContainer( this );

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        pad1 = new GamepadEx(gamepad1);
        angleController = new AngleController(hardwareMap);
        slideController = new SlideController(hardwareMap);
    //    wristController = new WristController(hardwareMap);

        btnPressCmd(Button.X, new InstantCommand( () -> angleController.setDegreesAboveHorizontal( ArmConstants.POSITION1 ) ) );
        btnPressCmd(Button.Y, new InstantCommand( () -> angleController.setDegreesAboveHorizontal( ArmConstants.POSITION2 ) ) );
        btnPressCmd(Button.B, new InstantCommand( () -> angleController.setDegreesAboveHorizontal( ArmConstants.POSITION3 ) ) );

        btnPressCmd(Button.DPAD_UP, new InstantCommand( () -> { angleController.adjustSetpoint( ArmConstants.NUDGE_DEGREES); } ) );
        btnPressCmd(Button.DPAD_DOWN, new InstantCommand( () -> { angleController.adjustSetpoint( -ArmConstants.NUDGE_DEGREES); } ) );
        btnPressCmd(Button.A, new InstantCommand( angleController::returnToHome ) );

        btnPressCmd(Button.DPAD_LEFT, new InstantCommand( () -> { slideController.setController(SlideConstants.SLIDE_CRUISE); } ) );
        btnPressCmd(Button.DPAD_RIGHT, new InstantCommand( () -> { slideController.setController(SlideConstants.SLIDE_MAX); } ) );

        btnPressCmd(Button.LEFT_BUMPER, new InstantCommand( () -> angleController.disableMotor(true)));
        btnPressCmd(Button.RIGHT_BUMPER, new InstantCommand( () -> angleController.disableMotor(false)));

        btnPressCmd(Button.BACK, new InstantCommand( angleController::stop ) );


    //    wristController.setOffsetDegrees( -ArmConstants.ARM_HOME_DEGREES_BELOW_HORIZONTAL );
    //    wristController.setAngleAboveHorizontal(0);

        schedule(new RunCommand(() -> {

            double power = pad1.getLeftY() / 1.0;
            if (Math.abs(power) >= 0.05) {
                angleController.setDirectPower(power);
            } else {
                if (angleController.isIdle()) angleController.setDirectPower(0);
            }

            slideController.update();
            angleController.setExtension( slideController.currentPosition() / SlideConstants.SLIDE_MAX );
            angleController.update();
  //          wristController.setOffsetDegrees(angleController.getDegreesAboveHorizontal());
  //          wristController.update();
            telemetry.addLine("Gamepad1: A=stop,X=POS0,Y=POS1,B=POS2,DPAD=Nudge arm angle");
            telemetry.addLine();
            angleController.debugInfo().forEach( (k,v) -> telemetry.addData(k,v) );
            telemetry.addData("Extension", "%1.2f",slideController.currentPosition() / SlideConstants.SLIDE_MAX );
            telemetry.update();
        }));
    }
}
