package org.firstinspires.ftc.teamcode.subsystems;


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

    private final PIDController mPIDExtention = new PIDController(Constants.BrasConstants.kPExtention, 0, 0);

    private double mPosRotationTarget;

    private double mRotationBrasInit = 0;

    private boolean isRotationCalibrated = false;

    private double mExTarget;

    private double minExtention;

    private boolean isExtentionCalibrated = false;

    public BrasSubsystem() {
        m_motorRotation.setInverted(false);
        m_MoteurExtention.setInverted(true);

        mPIDBrasRotation.setTolerance(Constants.BrasConstants.kToleranceRotation);
        mPIDExtention.setTolerance(Constants.BrasConstants.kToleranceExtention);
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

        DriverStationJNI.getTelemetry().addData("mExTarget", mExTarget);
        DriverStationJNI.getTelemetry().addData("ExtCurrentPosition", getExtentionPosition());
        DriverStationJNI.getTelemetry().addData("minExtention", minExtention);
        DriverStationJNI.getTelemetry().addData("minExtention+kmaxExt", minExtention + Constants.BrasConstants.kmaxExt);

        double ConsigneExtention = mPIDExtention.calculate(getExtentionPosition(), mExTarget);
        if (Math.abs(ConsigneExtention) > Constants.MaxSpeeds.kmaxExtentionSpeed) {
            ConsigneExtention = Math.signum(ConsigneExtention) * Constants.MaxSpeeds.kmaxExtentionSpeed;
        }
        DriverStationJNI.getTelemetry().addData("CONSIGNE EXTENTION", ConsigneExtention);
        m_MoteurExtention.set(ConsigneExtention);

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

    public boolean isExtentionAtSetPoint() {
        return mPIDExtention.atSetpoint();
    }

    public void incrementTargetRotation(double deltaTarget) {
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

    public void setTargetRotation(double target) {
        if (isRotationCalibrated) {
            mPosRotationTarget = mRotationBrasInit + target;
        }
    }

    public void stopRotation() {
        mPosRotationTarget = getPositionRotation();
    }

    public void extention(double deltaEx) {
        mExTarget += deltaEx;
        if (isExtentionCalibrated) {
            if (deltaEx < 0 && getExtentionPosition() <= minExtention) {
                stopExtention();
            }
            if (deltaEx > 0 && getExtentionPosition() >= minExtention + Constants.BrasConstants.kmaxExt) {
                stopExtention();
            }
        }
        if (deltaEx < 0 && isTouchExtention()) {
            stopExtention();
        }
    }

    public void setTargetExtention(double target) {
        if (isExtentionCalibrated) {
            mExTarget = minExtention + target;
        }
    }

    public double getExtentionPosition() {
        return m_MoteurExtention.getCurrentPosition();
    }

    public void stopExtention() {
        mExTarget = getExtentionPosition();
    }

    public void calibreExtention() {
        stopExtention();
        minExtention = getExtentionPosition();
        isExtentionCalibrated = true;
    }

}



