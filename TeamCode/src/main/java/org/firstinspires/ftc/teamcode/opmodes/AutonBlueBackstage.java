package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.BLUE;

@Autonomous(name="Auton BLUE BACKSTAGE", preselectTeleOp = "Drive Robot")
public class AutonBlueBackstage extends Auton {

    @Override
    protected void setAssumptions() {
        super.setAssumptions();
        forceColor = BLUE;
        forceBackstage = true;
    }
}