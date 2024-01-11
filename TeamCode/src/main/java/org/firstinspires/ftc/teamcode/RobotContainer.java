package org.firstinspires.ftc.teamcode;

import android.util.ArrayMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.commands.ArmStartPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToCruiseCommand;
import org.firstinspires.ftc.teamcode.commands.ArmToGroundCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryBuildCommand;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.AutonConstants;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SlideConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Map;
import java.util.function.Function;

public class RobotContainer {

    // The OpMode the robot is running
    public CommandOpMode opMode;

    // Controller
    private final GamepadEx m_gamepad1;
    private final GamepadEx m_gamepad2;

    // Drivetrain
    private final MecanumDriveSubsystem m_drivetrain;

    // Other subsystems to come..
    private final ArmSubsystem m_arm;
    private final DroneSubsystem m_drone;

    public Map<String, Object> m_telemetryItems;

    private TelemetryPacket m_telemPacket;

    private Telemetry telemetry;

    // vision processors
    public VisionPortal visionPortal = null;
    private TeamPropDetector visionPropDetector = null;
    private AprilTagProcessor visionAprilTag = null;

    private HardwareMap hardwareMap;

    public RobotContainer(CommandOpMode opMode) {

        this.telemetry = opMode.telemetry;
        this.hardwareMap = opMode.hardwareMap;
        m_telemPacket = new TelemetryPacket();
        m_telemetryItems = new ArrayMap<String, Object>();

        m_drivetrain = new MecanumDriveSubsystem(new SampleMecanumDrive(hardwareMap, this), false);
        m_drivetrain.getLocalizer().setPoseEstimate( PoseStorage.currentPose );

        // Bulks reads enabled in SampleMecanumDrive

        m_gamepad1 = new GamepadEx(opMode.gamepad1);
        m_gamepad2 = new GamepadEx(opMode.gamepad2);

        m_arm = new ArmSubsystem(hardwareMap, this);
        m_drone = new DroneSubsystem(hardwareMap, this);

        m_drivetrain.setDefaultCommand( new RunCommand( () -> { m_drivetrain.stop(); m_drivetrain.update(); } , m_drivetrain ) );

    }

    public GamepadEx getGamepad1() {
        return m_gamepad1;
    }
    public GamepadEx getGamepad2() {
        return m_gamepad2;
    }
    public MecanumDriveSubsystem getDrivetrain() { return m_drivetrain; }
    public ArmSubsystem getArm() { return m_arm; }
    public DroneSubsystem getDroneSubsystem() { return m_drone; }


    public void addTelem(String name, Object value) {
        m_telemetryItems.put(name, value);
    }

    public TelemetryPacket getTelemPacket() {
        return m_telemPacket;
    }

    public void sendTelem(FtcDashboard dashboard) {
        if (dashboard != null) {
            m_telemPacket.putAll( m_telemetryItems );
            dashboard.sendTelemetryPacket( m_telemPacket);
            m_telemPacket = new TelemetryPacket();
        }
    }

    public AprilTagProcessor aprilTagProcessor() {
        // create april tag processor if not created
        if (visionAprilTag == null) {
            // Set up apriltag
            // Create the AprilTag processor.
            visionAprilTag = new AprilTagProcessor.Builder()

                    // The following default settings are available to un-comment and edit as needed.
                    //.setDrawAxes(false)
                    //.setDrawCubeProjection(false)
                    //.setDrawTagOutline(true)
                    //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                    //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                    //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                    // == CAMERA CALIBRATION ==
                    // If you do not manually specify calibration parameters, the SDK will attempt
                    // to load a predefined calibration for your camera.
                    //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                    // ... these parameters are fx, fy, cx, cy.

                    .build();

        }
        return visionAprilTag;
    }

    public TeamPropDetector teamPropProcessor() {
        // create april tag processor if not created
        if (visionPropDetector == null) {
            // Set up apriltag
            // Create the AprilTag processor.
            visionPropDetector = new TeamPropDetector( telemetry );

        }
        return visionPropDetector;
    }

    public void startVisionProcessor(VisionProcessor vp) {
        if (visionPortal != null) visionPortal.setProcessorEnabled(vp, true);
    }

    public void stopVisionProcessor(VisionProcessor vp) {
        if (visionPortal != null) visionPortal.setProcessorEnabled(vp, false);
    }

    public boolean isVisionProcessorRunning(VisionProcessor vp) {
        return visionPortal.getProcessorEnabled(vp);
    }

    /**
     * Initialize the Vision portal with given processor enabled
     */
    public void initVision() {

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor( aprilTagProcessor() );
        builder.addProcessor( teamPropProcessor() );

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        visionPortal.setProcessorEnabled(visionAprilTag, false);
        visionPortal.setProcessorEnabled(visionPropDetector, false);

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void stopVision() {
        if (visionPortal != null) {
            stopVisionProcessor(visionPropDetector);
            stopVisionProcessor(visionAprilTag);

            visionPortal.close();
            visionPortal = null;
        }
    }

    public CommandFactory commandFactory() {
        return new CommandFactory(this);
    }

    public class CommandFactory {

        private RobotContainer robot;

        public CommandFactory(RobotContainer robot) {
            this.robot = robot;
        }

        public Command armToGround() {
            return new ArmToGroundCommand(robot.getArm());
        }

        public Command homeSlide() {
            return new SequentialCommandGroup(
                    new InstantCommand( () -> robot.getArm().homeSlide() ),
                    new WaitUntilCommand( () -> robot.getArm().slideIdle() )
            );
        }

        public Command armToCruise() {
            return new ArmToCruiseCommand(robot.getArm());
        }

        public Command armToStartPos() {
            return new ArmStartPositionCommand(robot.getArm());
        }

        public TrajectoryBuildCommand buildTraj(Function<TrajectorySequenceBuilder, TrajectorySequence> buildFunction) {
            return new TrajectoryBuildCommand(robot.getDrivetrain(), buildFunction);
        }

        public InstantCommand openGrab() {
            return new InstantCommand( () -> robot.getArm().grabOpen(), robot.getArm() );
        }

        public InstantCommand closeGrab() {
            return new InstantCommand( () -> robot.getArm().grabClose(), robot.getArm() );
        }

        public Command waitMillisecs(int ms) {
            return new RunCommand( () -> {} ).withTimeout(ms);
        }

        public Command raiseOnePixelFromGround() {
            return new SequentialCommandGroup(
                new InstantCommand(() -> { robot.getArm().setSlidePosition(SlideConstants.SLIDE_GROUND - SlideConstants.SLIDE_PIXEL_HEIGHT); } ),
                new WaitUntilCommand( () -> robot.getArm().slideCloseToPos() )
            );
        }

        public Command armToAprilTagScanPosition() {
            return new SequentialCommandGroup(
                    new InstantCommand( () -> robot.getArm().goToPosition( AutonConstants.ARM_APRILTAGSCAN_POSITION) ),
                    new InstantCommand( () -> { robot.getArm().setWristRelativeAngle(AutonConstants.WRIST_RELEASE_POSITION); } ),
                    new InstantCommand( () -> { robot.getArm().setSlidePosition(AutonConstants.SLIDE_APRILTAGSCAN_POSITION );})
            );
        }

        public Command raiseOutandRelease() {
            return new SequentialCommandGroup(
                    new InstantCommand( () -> robot.getArm().goToPosition(AutonConstants.ARM_RELEASE_POSITION) ),
                    new InstantCommand( () -> { robot.getArm().setWristRelativeAngle(AutonConstants.WRIST_RELEASE_POSITION); } ),
                    new InstantCommand( () -> { robot.getArm().setSlidePosition(AutonConstants.SLIDE_RELEASE_POSITION );}),
                    new WaitUntilCommand( () -> robot.getArm().slideCloseToPos() ),
         //             new WaitUntilCommand( () -> !robot.getDrivetrain().isBusy() ),
                    openGrab(),
                    waitMillisecs( (int)AutonConstants.PIXEL_RELEASE_DELAY_MS ),
                    new InstantCommand( () -> { robot.getArm().nudgeSlidePosition( -1800 );}),
                    new WaitUntilCommand( () -> robot.getArm().slideCloseToPos() )
                    );

        }

    }
}
