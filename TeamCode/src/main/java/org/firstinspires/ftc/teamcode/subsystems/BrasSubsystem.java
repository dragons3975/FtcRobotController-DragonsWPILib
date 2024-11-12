package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    //private final FtcMotor m_motorRotation = new FtcMotor("bras");


    private final PIDController mPIDBrasRotation = new PIDController(Constants.BrasConstants.kPRotation, 0, 0);

    private final PIDController mPIDExtention = new PIDController(Constants.BrasConstants.kPExtention, 0, 0);

    private double mPosRotationTarget;

    private double mRotationBrasInit = 0;

    private boolean isRotationCalibrated = false;

    private double mExTarget;

    private double minExtention;

    private boolean isExtentionCalibrated = false;

    public BrasSubsystem() {
        //m_motorRotation.setInverted(false);

        mPIDBrasRotation.setTolerance(Constants.BrasConstants.kToleranceRotation);
        //stopRotation();
    }

    @Override
    public void periodic() {

        //double consigne = mPIDBrasRotation.calculate(getPositionRotation(), mPosRotationTarget);
        //if (Math.abs(consigne) > Constants.MaxSpeeds.kmaxRotationSpeed) {
        //    consigne = Math.signum(consigne) * Constants.MaxSpeeds.kmaxRotationSpeed;
        //}
        //DriverStationJNI.getTelemetry().addData("CONSIGNE DU BRAS", consigne);
        //m_motorRotation.set(consigne);

    }



    public boolean isRotationAtSetPoint() {
        return mPIDBrasRotation.atSetpoint();
    }

    public boolean isExtentionAtSetPoint() {
        return mPIDExtention.atSetpoint();
    }

    public void incrementTargetRotation(double deltaTarget) {
        mPosRotationTarget += deltaTarget;
    }

    //public double getPositionRotation() {
    //    return m_motorRotation.getCurrentPosition();
    //}

    //public void calibreRotationBras() {
    //    stopRotation();
    //    mRotationBrasInit = getPositionRotation();
    //    isRotationCalibrated = true;
    //}

    //public void setTargetRotation(double target) {
    //    if (isRotationCalibrated) {
    //        mPosRotationTarget = mRotationBrasInit + target;
    //    }
    //}

    //public void stopRotation() {
   //     mPosRotationTarget = getPositionRotation();
    //}

}



