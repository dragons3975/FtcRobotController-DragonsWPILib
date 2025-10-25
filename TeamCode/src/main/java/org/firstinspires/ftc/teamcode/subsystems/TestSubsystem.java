package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestSubsystem extends Subsystem {

    private final FtcMotor mMotorTest = new FtcMotor("test");
    private final FtcTouchSensor mTouchSensor = new FtcTouchSensor("touch");
    private boolean mTouchBool;
    
    public TestSubsystem() {
    }

    @Override
    public void periodic() {
        mTouchBool = mTouchSensor.getState();
        mTouchBool = !mTouchBool;
        DriverStationJNI.getTelemetry().addData("Touch sensor", mTouchSensor.getState());
        DriverStationJNI.getTelemetry().addData("mTouch", mTouchBool);
    }
    public boolean isFinDeCourse() {
        return mTouchBool;
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