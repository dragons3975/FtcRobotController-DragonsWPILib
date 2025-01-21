package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurCordeSubsystem extends Subsystem {

    private final FtcMotor mMotorCorde = new FtcMotor("corde");//1

    PIDController mPidCorde = new PIDController(-0.003, 0, 0);
    //private double mSpeed = 0;
    private double mSpeedP = 0;

    private double mConsigneCorde = 0;

    public GrimpeurCordeSubsystem() {
        mPidCorde.setTolerance(100);
    }

    @Override
    public void periodic() {
        //mMotorGrimpeur.set(mSpeed);
        //mMotorPoulie.set(mSpeedP);
        double outputCorde = mPidCorde.calculate(mMotorCorde.getCurrentPosition(), mConsigneCorde);

        DriverStationJNI.getTelemetry().addData("CORDE position", mMotorCorde.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("CORDE consigne", mConsigneCorde);
        DriverStationJNI.getTelemetry().addData("CORDE Output", outputCorde);

        mMotorCorde.set(outputCorde);
    }

    public void stop() {
        //mSpeed = 0;
        mSpeedP = 0;
    }


    //public void setSpeed(double speed) {
    //    mSpeed = speed;
    //}


    public void incrementConsigneCorde(double incr) {
        mConsigneCorde += incr;
    }

    public boolean isConsigneCorde() {
        return mPidCorde.atSetpoint();
    }


    public void setConsigneCorde(double consigne) {
        mConsigneCorde = consigne;
    }

}
