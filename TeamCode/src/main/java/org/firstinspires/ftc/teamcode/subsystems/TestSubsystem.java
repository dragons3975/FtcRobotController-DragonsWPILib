package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class TestSubsystem extends Subsystem {

    private final FtcMotor mMotorTest = new FtcMotor("test");

    public TestSubsystem() {
    }

    @Override
    public void periodic() {

    }

    public void start() {
        mMotorTest.set(0.5);
    }
    public void pain() {
        mMotorTest.set(-0.5);
    }

    public void stop() {
        mMotorTest.stopMotor();
    }

}