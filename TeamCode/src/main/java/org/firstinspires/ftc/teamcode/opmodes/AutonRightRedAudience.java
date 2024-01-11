package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.RED;

@Autonomous(name="Auton RightSLOT RED AUDIENCE", preselectTeleOp = "Drive Robot")
public class AutonRightRedAudience extends Auton {

    @Override
    protected void setAssumptions() {
        super.setAssumptions();
        forceColor = RED;
        forceBackstage = false;
        leftSlot = false;
    }
}