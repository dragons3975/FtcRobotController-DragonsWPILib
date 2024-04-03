package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcMotor m_MotorBras = new FtcMotor("bras");
    private final FtcMotor mMotorBras2 = new FtcMotor("bras2");
    private final FtcMotor m_MoteurExtention = new FtcMotor("extention");

    private final PIDController mPIDBras = new PIDController(0.01, 0, 0);

    private final FtcTouchSensor mTouchSensor = new FtcTouchSensor("extention touch");

    private double m_posTarget;

    private double init = 0;

    private boolean isCalibrated = false;

    private double minExtention;

    public BrasSubsystem() {

        mPIDBras.setTolerance(10);
        m_MotorBras.setInverted(false);
        mMotorBras2.setInverted(true);
        m_MoteurExtention.setInverted(false);
        m_posTarget = getPosition();
    }

    @Override
    public void periodic() {
            double consigne = -mPIDBras.calculate(getPosition(), m_posTarget);
            DriverStationJNI.getTelemetry().addData("currentPosition", getPosition());
            DriverStationJNI.getTelemetry().addData("target", m_posTarget);
            DriverStationJNI.getTelemetry().addData("CONSIGNE DU BRAS", consigne);
            m_MotorBras.set(consigne);
            mMotorBras2.set(consigne);

            DriverStationJNI.getTelemetry().addData("ExtCurrentPosition", getExtentionPosition());
    }


    public void incrementTarget(double target) {
        m_posTarget += target;
        if (isCalibrated) {
            if (m_posTarget > init + Constants.ConstantsBras.kmax) {
                m_posTarget = init + Constants.ConstantsBras.kmax;
            }
            if (m_posTarget < init) {
                m_posTarget = init;
            }
        }
    }

    public double getPosition() {
        return  -m_MotorBras.getCurrentPosition();
    }

    public void setTarget(double target) {
        if (isCalibrated) {
            m_posTarget = init + target;
        }
    }

    public void extention(double ex) {
        DriverStationJNI.getTelemetry().addData("ex", ex);
        DriverStationJNI.getTelemetry().addData("isCalibrated", isCalibrated);
        DriverStationJNI.getTelemetry().addData("minExtention", minExtention);
        DriverStationJNI.getTelemetry().addData("minExtention+kmaxExt", minExtention + Constants.ConstantsBras.kmaxExt);

        if (ex > 0 && isCalibrated && getExtentionPosition() >= minExtention + Constants.ConstantsBras.kmaxExt) {
            m_MoteurExtention.set(0);
            return;
        }
        if (ex < 0 && isCalibrated && getExtentionPosition() <= minExtention) {
            m_MoteurExtention.set(0);
            return;
        }


        if (ex < 0) {
            if (getExtentionPosition() > minExtention){
                m_MoteurExtention.set(ex);
            }
        } else {
            m_MoteurExtention.set(ex);
        }
    }

    public double getExtentionPosition() {
        return -m_MoteurExtention.getCurrentPosition();
    }

    public void calibre() {
        init = getPosition();
        minExtention = getExtentionPosition();
        isCalibrated = true;
    }
}



