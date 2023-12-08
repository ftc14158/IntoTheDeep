package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.vision.TeamPropDetector.AllianceColor.BLUE;

@Autonomous(name="Auton BLUE AUDIENCE", preselectTeleOp = "Drive Robot")
public class AutonBlueAudience extends Auton {

    @Override
    protected void setAssumptions() {
        super.setAssumptions();
        forceColor = BLUE;
        forceBackstage = false;
    }
}