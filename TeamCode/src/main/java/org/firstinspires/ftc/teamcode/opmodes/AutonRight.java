package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.PerpetualCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseStorage;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.commands.AutoPlanCommand;
import org.firstinspires.ftc.teamcode.vision.TeamPropDetector;

import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.RED;

@Autonomous(name="Auton RightSLOT", preselectTeleOp = "Drive Robot")
public class AutonRight extends Auton {

    @Override
    protected void setAssumptions() {
        super.setAssumptions();
        leftSlot = false;
    }
}