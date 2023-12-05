package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.ArmControllerCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;

@TeleOp(group="drive", name = "Arm Test")
public class ArmTest extends CommandOpMode {

    private RobotContainer m_robot;

    /* X,Y of 0,0 is center of field.
   X increases to the right looking at center when viewed from behind red alliance wall
   Y increase going away from the center, when viewed from behind red alliance wall
   X and Y are in meters
   Heading is 0 degrees is looking along the positive X axis from the center of the field
   Heading increases turning counter clockwise when looking down at the field from above
    */

    @Override
    public void initialize() {
        m_robot = new RobotContainer( true, hardwareMap, telemetry,
                gamepad1, gamepad2, new Pose2d(-39,-63.5, Math.toRadians(90)));

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(1) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(2) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.BACK).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(3) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( () -> m_robot.getArm().goToLevel(0) ) );

        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(true) ) );
        m_robot.getGamepad2().getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setGrab(false)) );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgeSlidePosition( -100 ) ) );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand( () -> m_robot.getArm().nudgeSlidePosition(100) ) );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition( 500 ) )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition( 1000 ) )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand( () -> m_robot.getArm().setSlidePosition( 1500 ) )
        );

        m_robot.getGamepad1().getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand( () -> m_robot.getArm().stopSlide() )
        );


        FtcDashboard dashboard = FtcDashboard.getInstance();
        schedule( new PerpetualCommand( new RunCommand( () -> { m_robot.sendTelem( dashboard );})));

        schedule(new RunCommand(() -> {
            m_robot.getDrivetrain().update();
            telemetry.update();
        }));

    }

}
