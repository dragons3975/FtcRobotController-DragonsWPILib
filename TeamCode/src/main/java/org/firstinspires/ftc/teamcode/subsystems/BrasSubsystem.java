package org.firstinspires.ftc.teamcode.subsystems;


import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcMotor m_motorRotation = new FtcMotor("bras");
    private final FtcMotor m_MoteurExtention = new FtcMotor("extention");

    private final FtcTouchSensor mSensorPince = new FtcTouchSensor("pince touch");

    private final FtcTouchSensor mSensorExtention = new FtcTouchSensor("extention touch");


    private final PIDController mPIDBrasRotation = new PIDController(Constants.BrasConstants.kPRotation, 0, 0);



    private double mPosRotationTarget;

    private double mRotationBrasInit = 0;

    private boolean isRotationCalibrated = false;

    private double mEx;

    private double minExtention;

    private boolean isExtentionCalibrated = false;

    public BrasSubsystem() {
        m_motorRotation.setInverted(false);
        m_MoteurExtention.setInverted(true);

        mPIDBrasRotation.setTolerance(Constants.BrasConstants.kToleranceRotation);
        stopRotation();
    }

    @Override
    public void periodic() {

        DriverStationJNI.getTelemetry().addData("etat touch sensor extention", isTouchExtention());
        DriverStationJNI.getTelemetry().addData("etat touch sensor pince", isTouchPince());

        DriverStationJNI.getTelemetry().addData("getPositionRotation", getPositionRotation());
        DriverStationJNI.getTelemetry().addData("position init", mRotationBrasInit);
        DriverStationJNI.getTelemetry().addData("getPositionRotation-Init", getPositionRotation() - mRotationBrasInit);
        DriverStationJNI.getTelemetry().addData("m_posTarget", mPosRotationTarget);

        double consigne = mPIDBrasRotation.calculate(getPositionRotation(), mPosRotationTarget);
        if (Math.abs(consigne) > Constants.MaxSpeeds.kmaxRotationSpeed) {
            consigne = Math.signum(consigne) * Constants.MaxSpeeds.kmaxRotationSpeed;
        }
        DriverStationJNI.getTelemetry().addData("CONSIGNE DU BRAS", consigne);
        m_motorRotation.set(consigne);

        DriverStationJNI.getTelemetry().addData("ex output", mEx);
        DriverStationJNI.getTelemetry().addData("ExtCurrentPosition", getExtentionPosition());
        DriverStationJNI.getTelemetry().addData("minExtention", minExtention);
        DriverStationJNI.getTelemetry().addData("minExtention+kmaxExt", minExtention + Constants.BrasConstants.kmaxExt);
        if (isExtentionCalibrated) {
            if (mEx < 0 && getExtentionPosition() <= minExtention) {
                mEx = 0;
            }
            if (mEx > 0 && getExtentionPosition() >= minExtention + Constants.BrasConstants.kmaxExt) {
                mEx = 0;
            }
        }
        if (mEx < 0 && isTouchExtention()) {
            mEx = 0;
        }
        m_MoteurExtention.set(mEx);

    }

    public boolean isTouchExtention() {
        return !mSensorExtention.getState();
    }

    public boolean isTouchPince() {
        return !mSensorPince.getState();
    }

    public boolean isRotationAtSetPoint() {
        return mPIDBrasRotation.atSetpoint();
    }

    public void incrementTarget(double deltaTarget) {
        mPosRotationTarget += deltaTarget;
        if (isRotationCalibrated) {
            if (mPosRotationTarget > mRotationBrasInit + Constants.BrasConstants.kmax) {
                mPosRotationTarget = mRotationBrasInit + Constants.BrasConstants.kmax;
            }
            if (mPosRotationTarget < mRotationBrasInit) {
                mPosRotationTarget = mRotationBrasInit;
            }
        }
        else if (isTouchPince() && deltaTarget < 0) {
              stopRotation();
        }
    }

    public double getPositionRotation() {
        return m_motorRotation.getCurrentPosition();
    }

    public void calibreRotationBras() {
        stopRotation();
        mRotationBrasInit = getPositionRotation();
        isRotationCalibrated = true;
    }

    public void setTarget(double target) {
        if (isRotationCalibrated) {
            mPosRotationTarget = mRotationBrasInit + target;
        }
    }

    public void stopRotation() {
        mPosRotationTarget = getPositionRotation();
    }

    public void extention(double ex) {
        mEx = ex;
    }

    public void stopExtention() {
        extention(0);
    }


    public double getExtentionPosition() {
        return m_MoteurExtention.getCurrentPosition();
    }

    public void calibreExtention() {
        stopExtention();
        minExtention = getExtentionPosition();
        isExtentionCalibrated = true;
    }

}



