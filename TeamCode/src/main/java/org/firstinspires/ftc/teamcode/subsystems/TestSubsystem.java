package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class TestSubsystem extends Subsystem {

    private final FtcMotor mMotorTest = new FtcMotor("test");
    //private final FtcTouchSensor mTouchSensor = new FtcTouchSensor("touch");
   // private boolean mTouchBool;
    private final PIDController pid = new PIDController(0.05, 0, 0);
    private int mConsigne;
    private int mGetPositionActuelle;

    public TestSubsystem() {
        mConsigne = mMotorTest.getCurrentPosition();
    }

    @Override
    public void periodic() {
       // mTouchBool = mTouchSensor.getState();
     //   mTouchBool = !mTouchBool;
       // DriverStationJNI.getTelemetry().addData("Touch sensor", mTouchSensor.getState());
      //  DriverStationJNI.getTelemetry().addData("mTouch", mTouchBool);

        mGetPositionActuelle = mMotorTest.getCurrentPosition();
        DriverStationJNI.getTelemetry().addData("PositionActuelle", mMotorTest.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("PositionGet", mMotorTest.get());
        DriverStationJNI.getTelemetry().addData("PositionActVar", mGetPositionActuelle);

        double output = pid.calculate(mMotorTest.getCurrentPosition(), mConsigne);
        mMotorTest.set(output);



    }

    public boolean isFinDeCourse() {
     // return mTouchBool;
        return false;
    }

    public void setConsigne(int consigne) {
        mConsigne = consigne;
    }

    public int getPositionActuelle() {
        return mMotorTest.getCurrentPosition();
    }


    public void stop() {
        mConsigne = mGetPositionActuelle;
    }

}