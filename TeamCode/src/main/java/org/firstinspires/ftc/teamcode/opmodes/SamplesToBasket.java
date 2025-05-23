package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.AutoBasketCommand;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

@Autonomous(name="Samples To Basket", preselectTeleOp = "Drive Robot")
public class SamplesToBasket extends CommandOpMode {
    private RobotContainer robot = null;

    private TeamPropDetector visionProcessor1;

    protected static TeamPropDetector.AllianceColor forceColor = TeamPropDetector.AllianceColor.UNKNOWN;
    protected static Boolean forceBackstage = null;
    protected static boolean leftSlot;

    // Assume color and position unknown here, leave up to vision
    // system to detect.
    protected void setAssumptions() {
        forceColor = TeamPropDetector.AllianceColor.UNKNOWN;
        forceBackstage = null;
        leftSlot = true;
    }

    @Override
    public void initialize() {

        setAssumptions();

        // Create an FTC dashboard object to use to send back telemetry information
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        // Create the main robot object that contains the various
        // subsystems etc
                robot = new RobotContainer(this );

        // Create outr vision processor and start the vision portal
        //robot.initVision();
        //visionProcessor1 = robot.teamPropProcessor();
   //     robot.startVisionProcessor(visionProcessor1);

        // Create a command that will run every time in the loop to send back the telemetry to
        // the FTC dashboard
        schedule(new PerpetualCommand(new RunCommand(() -> {
            robot.sendTelem(dashboard);
        })));

     //   robot.getArm().grabClose();

        // schedule a command to decide what further commands to schedule...
        //schedule(new AutoPlanCommand(robot, visionProcessor1, forceColor, forceBackstage, leftSlot));
        schedule(new AutoBasketCommand(robot, TeamPropDetector.AllianceColor.RED ));
    }

    // Before reset at end of run, save current pose
    @Override
    public void reset() {

        if (robot != null) PoseStorage.currentPose = robot.getDrivetrain().getPoseEstimate();
        super.reset();
    }
}