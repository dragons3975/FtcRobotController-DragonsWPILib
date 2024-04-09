package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcMotor m_motorRotation = new FtcMotor("bras");
    private final FtcMotor m_motorRotation2 = new FtcMotor("bras2");
    private final FtcMotor m_MoteurExtention = new FtcMotor("extention");

    private final FtcTouchSensor mSensorPince = new FtcTouchSensor("pince touch");

    private final FtcTouchSensor mSensorExtention = new FtcTouchSensor("extention touch");


    private final PIDController mPIDBras = new PIDController(0.01, 0, 0);

    private double m_posTarget;

    private double init = 0;

    private boolean isRotationCalibrated = false;

    private double mEx;

    private double minExtention;

    private boolean isExtentionCalibrated;

    public BrasSubsystem() {

        mPIDBras.setTolerance(10);
        m_motorRotation.setInverted(false);
        m_motorRotation2.setInverted(true);
        m_MoteurExtention.setInverted(true);
        m_posTarget = getPositionRotation();
    }

    @Override
    public void periodic() {

        DriverStationJNI.getTelemetry().addData("etat touch sensor extention", mSensorExtention.getState());
        DriverStationJNI.getTelemetry().addData("etat touch sensor pince", mSensorPince.getState());

        DriverStationJNI.getTelemetry().addData("getPositionRotation", getPositionRotation());
        DriverStationJNI.getTelemetry().addData("m_posTarget", m_posTarget);
        double consigne = -mPIDBras.calculate(getPositionRotation(), m_posTarget);
        DriverStationJNI.getTelemetry().addData("CONSIGNE DU BRAS", consigne);
        m_motorRotation.set(consigne);
        m_motorRotation2.set(consigne);

        DriverStationJNI.getTelemetry().addData("ex output", mEx);
        DriverStationJNI.getTelemetry().addData("ExtCurrentPosition", getExtentionPosition());
        DriverStationJNI.getTelemetry().addData("minExtention", minExtention);
        DriverStationJNI.getTelemetry().addData("minExtention+kmaxExt", minExtention + Constants.ConstantsBras.kmaxExt);
        if (isExtentionCalibrated) {
            if (mEx < 0 && getExtentionPosition() <= minExtention) {
                mEx = 0;
            }
            if (mEx > 0 && getExtentionPosition() >= minExtention + Constants.ConstantsBras.kmaxExt) {
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

    public void incrementTarget(double target) {
        m_posTarget += target;
        if (isRotationCalibrated) {
            if (m_posTarget > init + Constants.ConstantsBras.kmax) {
                m_posTarget = init + Constants.ConstantsBras.kmax;
            }
            if (m_posTarget < init) {
                m_posTarget = init;
            }
        }
    }

    public double getPositionRotation() {
        return  -m_motorRotation.getCurrentPosition();
    }

    public void setTarget(double target) {
        if (isRotationCalibrated) {
            m_posTarget = init + target;
        }
    }

    public void extention(double ex) {
        mEx = ex;
    }

    public void stopExtention() {
        extention(0);
    }

    public void stopRotation() {
        setTarget(getPositionRotation());
    }

    public double getExtentionPosition() {
        return m_MoteurExtention.getCurrentPosition();
    }

    public void calibreExtention() {
        minExtention = getExtentionPosition();
        isExtentionCalibrated = true;
        stopExtention();
    }

    public void calibreBras() {
        init = getPositionRotation();
        isRotationCalibrated = true;
        stopRotation();
    }
}



