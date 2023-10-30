package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys.Button;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.MecanumDriveCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;

import java.util.Map;

public class RobotContainer {

    // Controller
    private final GamepadEx m_gamepad1;
    private final GamepadEx m_gamepad2;

    // Drivetrain
    private final MecanumDriveSubsystem m_drivetrain;

    // Other subsystems to come..
    private final ArmSubsystem m_arm;

    public Map<String, Object> m_telemetryItems;

    private TelemetryPacket m_telemPacket;

    public RobotContainer(boolean bTeleOp, HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2,
                          Pose2d startingPose) {

        m_telemPacket = new TelemetryPacket();
        m_telemetryItems = new ArrayMap<String, Object>();

        m_drivetrain = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap, this), false);

        m_drivetrain.getLocalizer().setPoseEstimate(startingPose);
        // Bulks reads enabled in SampleMecanumDrive

        m_gamepad1 = new GamepadEx(gamepad1);
        m_gamepad2 = new GamepadEx(gamepad2);

        m_arm = new ArmSubsystem(hardwareMap, this);

        if (bTeleOp) {
  //          configureButtonBindings();
            m_drivetrain.stop();

        } else {
            // stop drivetrain if not being commanded otherwise during loop
            m_drivetrain.setDefaultCommand( new RunCommand( m_drivetrain::stop, m_drivetrain ) );

            //intake.setDefaultCommand( new RunCommand(intake::stop, intake ));
        }

    }

    public GamepadEx getGamepad1() {
        return m_gamepad1;
    }
    public GamepadEx getGamepad2() {
        return m_gamepad2;
    }
    public MecanumDriveSubsystem getDrivetrain() { return m_drivetrain; }
    public ArmSubsystem getArm() { return m_arm; }

    private void configureButtonBindings() {
    //    m_drivetrain.setDefaultCommand(
    //            new MecanumDriveCommand()DefaultDifferentialDrive(drivetrain , () -> m_gamepad1.getLeftX() / 2,
    //                    () -> m_gamepad1.getLeftY()
    //            )
    //    );

        // make sure carousel stops if we are not telling it to do anything else
    //    carousel.setDefaultCommand( new RunCommand( carousel::stop, carousel) );

    //    assignActionButtons(m_gamepad1);
    //    assignActionButtons(m_gamepad2);



//                .whenReleased( new InstantCommand( () -> m_armSubsystem.setPower(0)));
/*
     m_gamepad1.getGamepadButton(Button.X).whenPressed( new SequentialCommandGroup(
             new DriveForward(m_driveSubsystem, 1).withTimeout(1000),
        new RotateToHeading(m_driveSubsystem, m_IMUSubsystem, 90, telemetry)
             ));

        // make button A drive forward for 4 seconds or until blue detected.
        m_gamepad1.getGamepadButton(Button.A).whenPressed(
            cc      new DriveForward(m_driveSubsystem, 1).withTimeout(4000)
                .interruptOn( m_colorSensors::isBlue )
        );



        m_gamepad1.getGamepadButton(Button.B).toggleWhenPressed( new RotateToHeading(m_driveSubsystem, m_IMUSubsystem, 90, telemetry));

     // Run each motor in turn when X button is pressed on keypad

     /*
     m_gamepad1.getGamepadButton(Button.X).whenPressed( new SequentialCommandGroup(

             new RunCommand( () -> m_driveSubsystem.setMotor(0, .5), m_driveSubsystem ).withTimeout(2000),
             new RunCommand( () -> m_driveSubsystem.setMotor( 0, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(1, .5), m_driveSubsystem ).withTimeout(2000),
                new RunCommand( () -> m_driveSubsystem.setMotor( 1, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(2, .5), m_driveSubsystem ).withTimeout(2000),
                new RunCommand( () -> m_driveSubsystem.setMotor( 2, 0), m_driveSubsystem ).withTimeout(1000),
        new RunCommand( () -> m_driveSubsystem.setMotor(3, .5), m_driveSubsystem ).withTimeout(2000),
                new InstantCommand( () -> m_driveSubsystem.setMotor( 3, 0), m_driveSubsystem )
     ) );
*/
    }
/*
    private void assignActionButtons(GamepadEx g) {

        g.getGamepadButton(Button.DPAD_UP).whenPressed(new InstantCommand(
                () -> arm.nudgePosition(20)));

        g.getGamepadButton(Button.DPAD_DOWN).whenPressed(new InstantCommand(
                () -> arm.nudgePosition(-20)));

        g.getGamepadButton(Button.A).whenPressed(new InstantCommand(
                () -> arm.goToLevel(1)));

        g.getGamepadButton(Button.B).whenPressed(new InstantCommand(
                () -> arm.goToLevel(2)));

        g.getGamepadButton(Button.Y).whenPressed(new InstantCommand(
                () -> arm.goToLevel(3)));

        // turn off arm motor so falls to bottom position
        g.getGamepadButton(Button.X).whenPressed(new InstantCommand(
                () -> arm.goToLevel(0) ) );

        g.getGamepadButton(Button.LEFT_BUMPER).toggleWhenPressed(
                new InstantCommand(intake::suck),
                new InstantCommand(intake::stop)
        );

        g.getGamepadButton(Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(intake::eject)
        ).whenReleased(intake::stop);


        g.getGamepadButton(Button.DPAD_LEFT).whenHeld(
                new RunCommand(carousel::forward, carousel)
        );

        g.getGamepadButton(Button.DPAD_RIGHT).whenHeld(
                new RunCommand(carousel::backward, carousel)
        );

    }
*/

    public void addTelem(String name, Object value) {
        m_telemetryItems.put(name, value);
    }

    public TelemetryPacket getTelemPacket() {
        return m_telemPacket;
    }

    public void sendTelem(FtcDashboard dashboard) {
        m_telemPacket.putAll( m_telemetryItems );
        dashboard.sendTelemetryPacket( m_telemPacket);
        m_telemPacket = new TelemetryPacket();

    }
}
