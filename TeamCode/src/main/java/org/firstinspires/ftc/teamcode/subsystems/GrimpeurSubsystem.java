package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurSubsystem extends Subsystem {
    private final FtcMotor mMotorPoulie = new FtcMotor("poulie");//1
    PIDController mPid = new PIDController(0.003, 0, 0);
    //private double mSpeed = 0;
    private double mSpeedP = 0;
    private double mConsigne = 0;

    public GrimpeurSubsystem() {
    }

    @Override
    public void periodic() {
        //mMotorGrimpeur.set(mSpeed);
        //mMotorPoulie.set(mSpeedP);
        double output = mPid.calculate(mMotorPoulie.getCurrentPosition(), mConsigne);
        DriverStationJNI.getTelemetry().addData("POULIE position", mMotorPoulie.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("POULIE conigne", mConsigne);
        DriverStationJNI.getTelemetry().addData("Output", output);
        mMotorPoulie.set(output);
    }

    public void stop() {
        //mSpeed = 0;
        mSpeedP = 0;
    }


    //public void setSpeed(double speed) {
    //    mSpeed = speed;
    //}


    public void setSpeedP(double speed) {
        mSpeedP = speed;
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
