package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public class ExtensionSubsystem extends Subsystem {

    private final FtcMotor mMotorCalib = new FtcMotor("Extension");

    private final FtcTouchSensor mFtcTouchSensor = new FtcTouchSensor("touch");


    PIDController mPid = new PIDController(0.01 , 0, 0);

    private int mConsigne = 0;

    private double mSpeed = 0;

    private int mTachoCalib = 0;

    private double mDistance = 0;

    public ExtensionSubsystem() {
        mMotorCalib.setInverted(false);

        mPid.setTolerance(3);
    }

    @Override
    public void periodic() {

        //if (mFtcTouchSensor.getState()) {
        //    mSpeed = 0;
        //}
        mMotorCalib.set(mSpeed);

        mDistance = ((1.0/Constants.ExtensionConstants.kTachoCount)*getCalibratedTacho())*(Constants.ExtensionConstants.kCirconference);

        DriverStationJNI.getTelemetry().addData("Encodeur Calib", getCalibratedTacho());
        DriverStationJNI.getTelemetry().addData("TouchSensor", mFtcTouchSensor.getState());
        DriverStationJNI.getTelemetry().addData("Speed Calib", mSpeed);
        DriverStationJNI.getTelemetry().addData("Distance mm", mDistance);

        double output = mPid.calculate(getCalibratedTacho(), mConsigne);

        //if(output >= 0.7) {
        //    output = 0.7;
        //}
        //if(output <= 0.0) {
        //   output = 0.0;
        //}

        DriverStationJNI.getTelemetry().addData("IsConsigne t/f", isConsigne());
        DriverStationJNI.getTelemetry().addData("Output", output);
        mMotorCalib.set(output);

    }

    public void testBouger() {
        mMotorCalib.set(1);
    }

    public void StartCalibration() {
        mSpeed = -0.5;
        //mTachoCalib = (mArduinoCalib.getTachoCount()-mArduinoCalib.getTachoCount());
    }

    public void CalibrationZero() {
        mTachoCalib = mMotorCalib.getCurrentPosition();
    }

    private int getCalibratedTacho() {
        return mMotorCalib.getCurrentPosition();// - mTachoCalib;
    }

    public void stop() {
        mConsigne = getCalibratedTacho();
    }

    public void setSpeed(double speed) {
        mSpeed = speed;
    }

    public boolean isCalibrationFinished() {
        return mFtcTouchSensor.getState();
    }

    public double getDistanceXcm() {
        return mDistance;
    }

    public int getTacho() {
        return mMotorCalib.getCurrentPosition();
    }

    public void setConsigne(int consigne) {
        mConsigne = consigne;
    }

    public boolean isConsigne() {
        return mPid.atSetpoint();
    }


}


