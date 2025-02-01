package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceExtensionSubsystem extends Subsystem {

    private final FtcServo mServoRotation = new FtcServo("RotationPinceExt");//2

    private final FtcServo mServoPosition = new FtcServo("PositionPinceExt");//3

    private final FtcServo mServoPince = new FtcServo("PinceExt");//4

    private double mAngle = 0;
    private double mPositionAngle = 1;
    private double mRotationAngle = 1;

    public PinceExtensionSubsystem() {
        openPince();
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Servo Pince extend Angle", mAngle);
        DriverStationJNI.getTelemetry().addData("Servo Pince extend Position", mPositionAngle);
        DriverStationJNI.getTelemetry().addData("Servo Pince rotation", mRotationAngle);
        mServoPince.setPosition(mAngle);
        mServoPosition.setPosition(mPositionAngle);
        mServoRotation.setPosition(mRotationAngle);
    }

    public void openPince() {
        mAngle = 1;
    }

    public void closePince() {
        mAngle = 0.78;
    }

    public boolean isOpen() {
        return (mAngle == 1);
    }

    public void setPositionAngle(double position) {
        mPositionAngle = position;
    }

    public void incrementPositionAngle(double position) {
        mPositionAngle += position;
    }

    public void incrementRotationAngle(double rotation) {
        mRotationAngle += rotation;
    }

    public void setRotationAngle(double rotation) {
        mRotationAngle = rotation;
    }


}


