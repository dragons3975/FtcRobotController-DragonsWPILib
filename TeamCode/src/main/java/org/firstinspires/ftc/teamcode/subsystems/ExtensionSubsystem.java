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


    PIDController mPid = new PIDController(0.015, 0, 0);

    private int mConsigne = 0;

    private double mSpeed = 0;

    private double mDistance = 0;

    private boolean isCalib = false;

    private int mTachoCalib = 0;

    public ExtensionSubsystem() {
        mMotorCalib.setInverted(false);

        mPid.setTolerance(3);
    }

    @Override
    public void periodic() {

        //if (mFtcTouchSensor.getState()) {
        //    mSpeed = 0;
        //}

        mDistance = ((1.0/Constants.ExtensionConstants.kTachoCount)*getCalibratedTacho())*(Constants.ExtensionConstants.kCirconference);

        DriverStationJNI.getTelemetry().addData("Extend Position", getCalibratedTacho());
        DriverStationJNI.getTelemetry().addData("Extend consigne", mConsigne);

        if (mConsigne >= 1700) {
            mConsigne = 1699;
        }

        double output = mPid.calculate(getCalibratedTacho(), mConsigne);

        //if(output >= 0.7) {
        //    output = 0.7;
        //}
        //if(output <= 0.0) {
        //   output = 0.0;
        //}

        DriverStationJNI.getTelemetry().addData("IsConsigne t/f", isConsigne());
        DriverStationJNI.getTelemetry().addData("Extend Output", output);

        mMotorCalib.set(output);

    }


    public void CalibrationZero() {
        mMotorCalib.resetEncoder();
        mConsigne = 0;
    }

    public int getCalibratedTacho() {
        return mMotorCalib.getCurrentPosition();
    }


    public boolean isCalibrationFinished() {
        return mFtcTouchSensor.getState();
    }


    public void setConsigne(int consigne) {
        mConsigne = consigne;
    }

    public void incrementConsigne(double incr) {
        mConsigne += incr;
    }

    public boolean isConsigne() {
        return mPid.atSetpoint();
    }


}


