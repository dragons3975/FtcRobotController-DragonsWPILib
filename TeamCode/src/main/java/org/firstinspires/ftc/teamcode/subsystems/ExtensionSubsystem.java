package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.firstinspires.ftc.teamcode.Constants;

public class ExtensionSubsystem extends Subsystem {

    private final FtcMotor mMotorCalib = new FtcMotor("Extension");

    private final FtcTouchSensor mFtcTouchSensor = new FtcTouchSensor("touch");

    private double mSpeed = 0;

    private int mTachoCalib = 0;

    private double mDistance = 0;

    public ExtensionSubsystem() {
        mMotorCalib.setInverted(true);
    }

    @Override
    public void periodic() {

        if (mFtcTouchSensor.getState()) {
            mSpeed = 0;
        }
        mMotorCalib.set(mSpeed);

        mDistance = ((1.0/Constants.ExtensionConstants.kTachoCount)*getCalibratedTacho())*(Constants.ExtensionConstants.kCirconference);

        DriverStationJNI.Telemetry.putNumber("Encodeur Calib", getCalibratedTacho());
        DriverStationJNI.Telemetry.putBoolean("TouchSensor", mFtcTouchSensor.getState());
        DriverStationJNI.Telemetry.putNumber("Speed Calib", mSpeed);
        DriverStationJNI.Telemetry.putNumber("Distance mm", mDistance);
    }

    public void StartCalibration() {
        mSpeed = -0.5;
        //mTachoCalib = (mArduinoCalib.getTachoCount()-mArduinoCalib.getTachoCount());
    }

    public void CalibrationZero() {
        mTachoCalib = mMotorCalib.getCurrentPosition();
    }

    private int getCalibratedTacho() {
        return mMotorCalib.getCurrentPosition() - mTachoCalib;
    }

    public void stop() {
        mSpeed = 0;
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

}


