package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.AutoPlanCommand;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

@Autonomous(name="Auton BLUE BACKSTAGE")
public class AutonBlueBackstage extends CommandOpMode {
    private RobotContainer m_robot;

    private TeamPropDetector visionProcessor1;

    private static TeamPropDetector.AllianceColor forceColor = TeamPropDetector.AllianceColor.BLUE;
    private static Boolean forceBackstage = true;

    @Override
    public void initialize() {

        // Create an FTC dashboard object to use to send back telemetry information
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Create the main robot object that contains the various
        // subsystems etc
        m_robot = new RobotContainer(false, hardwareMap, telemetry,
                gamepad1, gamepad2, new Pose2d(-39, -62.125, Math.toRadians(90)));

        // Create outr vision processor and start the vision portal
        m_robot.initVision();
        visionProcessor1 = m_robot.teamPropProcessor();
        m_robot.startVisionProcessor(visionProcessor1);

        // Create a command that will run every time in the loop to send back the telemetry to
        // the FTC dashboard
        schedule(new PerpetualCommand(new RunCommand(() -> {
            m_robot.sendTelem(dashboard);
        })));

        m_robot.getArm().grabClose();

        // schedule a command to decide what further commands to schedule...
        schedule(new AutoPlanCommand(m_robot, visionProcessor1, forceColor, forceBackstage));
        // schedule(new AutoTestCommand(m_robot, visionProcessor1));
    }
}