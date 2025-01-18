package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LiftSubsystem extends Subsystem {

    private final FtcMotor m_motorRotation = new FtcMotor("lift");

    private final PIDController mPIDBrasRotation = new PIDController(Constants.LiftConstants.kPRotation, 0, 0);

    private double mPosRotationTarget;


    private double mRotationBrasInit = 0;

    private boolean isRotationCalibrated = false;



    public LiftSubsystem() {
//        m_motorRotation.setInverted(true);

        mPIDBrasRotation.setTolerance(Constants.LiftConstants.kToleranceRotation);
        //stopRotation();
    }

    @Override
    public void periodic() {

        double consigne = mPIDBrasRotation.calculate(getPositionRotation(), mPosRotationTarget);
        if (Math.abs(consigne) > Constants.MaxSpeeds_lift.kmaxRotationSpeed) {
            consigne = Math.signum(consigne) * Constants.MaxSpeeds_lift.kmaxRotationSpeed;
        }
        DriverStationJNI.getTelemetry().addData("CONSIGNE DU LIFT", consigne);
        DriverStationJNI.getTelemetry().addData("LIFT POSITION", getPositionRotation());
        DriverStationJNI.getTelemetry().addData("TARGET LIFT", mPosRotationTarget);
        m_motorRotation.set(consigne);
    }



    public boolean isRotationAtSetPoint() {
        return mPIDBrasRotation.atSetpoint();
    }


    public void incrementTargetRotation(double deltaTarget) {
        mPosRotationTarget += deltaTarget;
        if(mPosRotationTarget >= Constants.LiftConstants.kMaximumPosition) {
            mPosRotationTarget = Constants.LiftConstants.kMaximumPosition;
        }
        if(mPosRotationTarget <= Constants.LiftConstants.kMinimumPosition) {
            mPosRotationTarget = Constants.LiftConstants.kMinimumPosition;
        }
    }

    public double getPositionRotation() {
        return m_motorRotation.getCurrentPosition();

    }



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



