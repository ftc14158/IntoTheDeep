package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotContainer;

public class DroneSubsystem extends SubsystemBase {

    // motor
    private MotorEx rollerMotor;

    // servo
    private Servo launchServo;

    private RobotContainer m_robot;

    public DroneSubsystem(HardwareMap hardwareMap, RobotContainer robot) {
        m_robot = robot;

//        rollerMotor = new MotorEx(hardwareMap, "launcher");
//        rollerMotor.setInverted(true);
//        rollerMotor.setRunMode(Motor.RunMode.RawPower);
        // rollerMotor.stopMotor();

        launchServo = hardwareMap.get(Servo.class, "launch");
        stop();
    }

    public void launch() {
        // Run motor to launch
  //      rollerMotor.set(-1);
        launchServo.setPosition( ArmConstants.DRONE_LAUNCH );
    }

    public void stop() {
//        rollerMotor.stopMotor();
        launchServo.setPosition( ArmConstants.DRONE_SET );
    }
}
