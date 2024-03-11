package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {


    private final FtcMotor mMotorBras = new FtcMotor("motor0");
    private final FtcMotor mMotorCoude = new FtcMotor("motor1");
    private final FtcServo mServoRotationPince = new FtcServo("servo1");


    private double mPosRotationPince = 0.5;
    private int mPosCoude;
    private int mPosBrasMoteur;

    public BrasSubsystem() {
        mPosBrasMoteur = mMotorBras.getCurrentPosition();
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Encodeur bras", mMotorBras.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("Variable Moteur", mPosBrasMoteur);
        DriverStationJNI.getTelemetry().addData("Encodeur PosCoude", mMotorCoude.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("Variable PosCoude", mPosCoude);
        DriverStationJNI.getTelemetry().addData("Variable PosRotationPince", mPosRotationPince);
        mMotorBras.setTargetPosition(mPosBrasMoteur, 0.15); //TODO tester si tiens power (devrait mais on sait pas)
        mMotorCoude.setTargetPosition(mPosCoude, 0.15);
        mServoRotationPince.setPosition(mPosRotationPince);

    }

    public void armPosition(int position){
        if (position == 0) { //TODO mettre les positions aux moteurs

        }
        if (position == 1) { //TODO mettre les positions aux moteurs

        }
        if (position == 2) { //TODO mettre les positions aux moteurs

        }
        if (position == 3) { //TODO mettre les positions aux moteurs

        }
    }
    public void armPosIncrement(int motorBras, int motorCoude, double servoRotationPince){
        mPosBrasMoteur += motorBras;
        mPosCoude += motorCoude;
        mPosRotationPince += servoRotationPince;
    }
    public void armGoTo(int motorBras, double servoRotationPince, int motorCoude){
        if (motorBras != -1){
            mPosBrasMoteur = motorBras;
        }
        if (motorCoude != -1){
            mPosCoude = motorCoude;
        }
        if (servoRotationPince != -1){
            mPosRotationPince = servoRotationPince;
        }
    }

}



