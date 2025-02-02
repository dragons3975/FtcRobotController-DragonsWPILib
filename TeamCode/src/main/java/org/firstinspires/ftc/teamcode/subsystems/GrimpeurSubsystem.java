package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurSubsystem extends Subsystem {
    //private final FtcMotor mMotorPlieur = new FtcMotor("plieur");//1
    PIDController mPidPlieur = new PIDController(0.004, 0, 0);
    //private double mSpeed = 0;
    private double mSpeedP = 0;
    private double mConsignePlieur = 0;

    public GrimpeurSubsystem() {
        mPidPlieur.setTolerance(50);
    }

    @Override
    public void periodic() {
        //mMotorGrimpeur.set(mSpeed);
        //mMotorPoulie.set(mSpeedP);
        //double output = mPidPlieur.calculate(mMotorPlieur.getCurrentPosition(), mConsignePlieur);

        //DriverStationJNI.getTelemetry().addData("PlIEUR position", mMotorPlieur.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("PLIEUR consigne", mConsignePlieur);
        //DriverStationJNI.getTelemetry().addData("PLIEUR Output", output);
        DriverStationJNI.getTelemetry().addData("PLIEUR is at setpoint", isConsignePlieur());

        //mMotorPlieur.set(output);
    }

    public void incrementConsignePlieur(double incr) {
        mConsignePlieur += incr;
    }

    public boolean isConsignePlieur() {
        return mPidPlieur.atSetpoint();
    }


    public void setConsignePlieur(double consigne) {
        mConsignePlieur = consigne;
    }
}
