package org.firstinspires.ftc.teamcode.subsystems.mechanism;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.subsystems.ArmConstants;
import org.firstinspires.ftc.teamcode.subsystems.MechanismMode;

import java.util.HashMap;
import java.util.Map;

import static com.arcrobotics.ftclib.util.MathUtils.clamp;
import static org.firstinspires.ftc.teamcode.subsystems.MechanismMode.*;

public class SlideController {

    private MotorEx motor;
    private TouchSensor limit;

    private MechanismMode mode = IDLE;

    private PIDController controller = null;

    private int currentPosition = 0;
    private int lastPosition = 0;
    private int positionDiff = 0;
    private boolean limitPressed = false;

    double error = 0;
    double power = 0;

    public SlideController(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, ArmConstants.SLIDE_MOTOR);
        limit = hardwareMap.get(TouchSensor.class, ArmConstants.SLIDE_LIMIT_SWITCH);
        controller = new PIDController(ArmConstants.SLIDE_KP, ArmConstants.SLIDE_KI, 0);

        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        motor.setInverted(true);
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(0);
        motor.resetEncoder();
        mode = RETURNING_TO_HOME;
    }

    private boolean limitCheck() {
        if (limitPressed) {
            motor.stopMotor();
            motor.resetEncoder();
            mode = IDLE;
        }
        return limitPressed;
    }

    public void update() {
        // Read the hardware
        currentPosition = motor.getCurrentPosition();
        limitPressed = limit.isPressed();

        positionDiff = Math.abs(currentPosition - lastPosition);
        lastPosition = currentPosition;

        switch(mode) {
            case POSITIONING:
                // Check limit only if moving toward limit switch. If limit was checked when moving away,
                // state would end immediately if starting from at limit (with limit switch pressed)
                if ((controller.getSetPoint() > currentPosition) || (!limitCheck())) {
                    // if very little movement since last check (less than 2 ticks), and position is without
                    // tolerance of desired, then stop motor and go to IDLE
                    if ((positionDiff < 2) && Math.abs(currentPosition - controller.getSetPoint()) < ArmConstants.SLIDE_TOLERANCE) {
                        // slideMotor.atTargetPosition()) {
                        motor.stopMotor();
                        mode = IDLE;
                    } else {
                        error = controller.calculate(currentPosition);
                        error = clamp(error, -1.0, 1.0);
                        power = error * ArmConstants.SLIDE_POWER_SIGN;
                        motor.set(power);
                    }
                }
                break;

            case RETURNING_TO_HOME:
                if (!limitCheck()) {
                    // run motor in reverse until limit
                    error = 0;
                    power =  -Math.signum(ArmConstants.SLIDE_POWER_SIGN) * ArmConstants.SLIDE_HOME_POWER;
                    motor.set( power );
                }
                break;

            case SETTLING_HOME:
                if (!limitCheck() ) {
                    if (currentPosition > 300) {
                        motor.set(-Math.signum(ArmConstants.SLIDE_POWER_SIGN) * Math.signum(ArmConstants.SLIDE_HOIST_POWER));
                    } else {
                            motor.set(-Math.signum(ArmConstants.SLIDE_POWER_SIGN) * ArmConstants.SLIDE_HOME_POWER);
                        }
                    }
                break;

            case IDLE:
                motor.stopMotor();
                motor.setRunMode(Motor.RunMode.RawPower);
                // after limit is pressed and motor stopped, it is possible shaft can still spin slightly,
                // so reset encoder to zero if it has gone slightly past
                // if (limitPressed && (currentPosition != 0)) motor.resetEncoder();
        }
    }

    public int currentPosition() {
        return currentPosition;
    }

    public void stop() {
        mode = IDLE;
    }

    public void hoist() {
            mode = SETTLING_HOME;
    }

    public void setPower(double power) {
//        if (mode == IDLE) motor.set(power);
    }

    public void setController(double position) {
        Log.i("ARM SLIDE", "Pos " + position);
        controller.setPID(ArmConstants.SLIDE_KP, ArmConstants.SLIDE_KI, ArmConstants.SLIDE_KD);
        controller.setSetPoint(position);
        motor.setRunMode(Motor.RunMode.RawPower);
        mode = POSITIONING;
    }

    public void returnToHome() {
        mode = RETURNING_TO_HOME;
    }

    public boolean busy() {
        return mode != IDLE;
    }

    public boolean idle() {
        return !busy();
    }

    public boolean closeToPos() {
        return (idle()) || (( Math.abs( controller.getSetPoint() - lastPosition)) < 100 );
    }
    public void nudgePosition(double delta) {
        setController( clamp( currentPosition + delta, 50, ArmConstants.SLIDE_MAX)  );
    }

    public Map<String, Object> debugInfo() {
        Map<String, Object> data = new HashMap<String, Object>();

        data.put("Mode", mode);
        data.put("Position", currentPosition);
        data.put("Set Point", controller.getSetPoint());
        data.put("PID Error", error);
        data.put("Applied Power", power);
        data.put("Limit Pressed", limitPressed);
        return data;
    }
}
