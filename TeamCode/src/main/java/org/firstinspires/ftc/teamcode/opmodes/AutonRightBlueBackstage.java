package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.BLUE;

@Autonomous(name="Auton RightSLOT BLUE BACKSTAGE", preselectTeleOp = "Drive Robot")
public class AutonRightBlueBackstage extends Auton {

    @Override
    protected void setAssumptions() {
        super.setAssumptions();
        leftSlot = false;
        forceColor = BLUE;
        forceBackstage = true;
    }
}