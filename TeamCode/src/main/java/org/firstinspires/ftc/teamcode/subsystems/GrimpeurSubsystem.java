package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurSubsystem extends Subsystem {

    //private final FtcMotor mMotorGrimpeur = new FtcMotor("grimpeur");
    private final FtcMotor mMotorPoulie = new FtcMotor("poulie");
    private double mSpeed = 0;
    private double mSpeedP = 0;

    public GrimpeurSubsystem() {
    }

    @Override
    public void periodic() {
        //mMotorGrimpeur.set(mSpeed);
        mMotorPoulie.set(mSpeedP);
    }

    public void stop() {
        mSpeed = 0;
        mSpeedP = 0;
    }


    public void setSpeed(double speed) {
        mSpeed = speed;
    }


    public void setSpeedP(double speed) {
        mSpeedP = speed;
    }
}
