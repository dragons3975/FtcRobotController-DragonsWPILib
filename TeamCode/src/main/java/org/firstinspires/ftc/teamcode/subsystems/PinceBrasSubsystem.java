package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class  PinceBrasSubsystem extends Subsystem {

    private final FtcServo mServoPince = new FtcServo("PinceBras");

    private final FtcServo mServoPincePosition = new FtcServo("PincePositionBras");

    private double mAngle = 0.9;
    private double mPositionAngle = 0.8;

    public PinceBrasSubsystem() {
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Servo Pince Angle", mAngle);
        DriverStationJNI.getTelemetry().addData("Servo Pince Position", mPositionAngle);
        mServoPincePosition.setPosition(mPositionAngle);
        mServoPince.setPosition(mAngle);
    }

    public void FermePince() {
        mAngle = 0.9;
    }

    public void OpenPince() {
        mAngle = 0.6;
    }

    public boolean isOpen() {
        return (mAngle == 0.9);
    }

    public void PositionPinceMax() {
        mPositionAngle = 0.8;
    }

    public void PositionPinceMin() {
        mPositionAngle = 0;
    }

    public boolean isPositionMax() {
        return (mPositionAngle == 0.8);
    }

    public void setPositionAngle(double position) {
        mPositionAngle = position;
    }

}