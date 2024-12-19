package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurSubsystem extends Subsystem {

    //private final FtcMotor mMotorGrimpeur = new FtcMotor("grimpeur");
    private final FtcMotor mMotorPoulie = new FtcMotor("poulie");//1
    PIDController mPid = new PIDController(0.01 , 0, 0);
    //private double mSpeed = 0;
    private double mSpeedP = 0;
    private int mConsigne = 0;

    public GrimpeurSubsystem() {
    }

    @Override
    public void periodic() {
        //mMotorGrimpeur.set(mSpeed);
        //mMotorPoulie.set(mSpeedP);
        double output = mPid.calculate(mMotorPoulie.getCurrentPosition(), mConsigne);
        DriverStationJNI.getTelemetry().addData("IsConsigne t/f poulie", isConsigne());
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

    public void incrementConsigne(int incr) {
        mConsigne += incr;
    }

    public boolean isConsigne() {
        return mPid.atSetpoint();
    }
}
