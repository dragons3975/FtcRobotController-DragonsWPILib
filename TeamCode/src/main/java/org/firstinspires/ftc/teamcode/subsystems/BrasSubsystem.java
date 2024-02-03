package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcMotor mMotorBras = new FtcMotor("bras");
    private int mPositionMin;
    private int mPositionMax;
    private int mPositionCurrent;


    public BrasSubsystem() {
        mPositionCurrent = mMotorBras.getCurrentPosition();
        mPositionMin = mPositionCurrent;
        mPositionMax = mPositionMin + 370;
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("consigne", mPositionCurrent);
        DriverStationJNI.getTelemetry().addData("capteur", mMotorBras.getCurrentPosition());
        mMotorBras.setTargetPosition(mPositionCurrent);
    }

    public void monte() {
        mPositionCurrent = mPositionCurrent + 2;
        if (mPositionCurrent > mPositionMax) {
            mPositionCurrent = mPositionMax;
        }
    }

    public void descend() {
        mPositionCurrent = mPositionCurrent - 2;
        if (mPositionCurrent < mPositionMin) {
            mPositionCurrent = mPositionMin;
        }
    }

    public void goToPosition(int position) {
        if (position > mPositionMin && position < mPositionMax) {
            mPositionCurrent = position;
        }
    }
}

