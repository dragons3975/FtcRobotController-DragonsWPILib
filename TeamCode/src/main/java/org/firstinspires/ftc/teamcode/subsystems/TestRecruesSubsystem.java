package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class TestRecruesSubsystem extends Subsystem {

    private final FtcMotor mMotorTest2 = new FtcMotor("Sushi");

    public TestRecruesSubsystem() {
    }

    @Override
    public void periodic() {

    }

    public void start() {
        mMotorTest2.set(0.5);
    }
    public void pain() {
        mMotorTest2.set(-0.5);
    }

    public void stop() {
        mMotorTest2.stopMotor();
    }

}