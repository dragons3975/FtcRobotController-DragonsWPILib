package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceExtensionSubsystem extends Subsystem {

    private final FtcServo mServoPince = new FtcServo("PinceExt");//3

    private final FtcServo mServoPosition = new FtcServo("PositionPinceExt");//2

    private final FtcServo mServoRotation = new FtcServo("RotationPinceExt");//4

    private double mAngle = 0;
    private double mPositionAngle = 0;
    private double mRotationAngle = 0;

    public PinceExtensionSubsystem() {
    }

    @Override
    public void periodic() {
        DriverStationJNI.Telemetry.putNumber("Servo Pince extend Angle", mAngle);
        DriverStationJNI.Telemetry.putNumber("Servo Pince  extend Position", mPositionAngle);
        mServoPince.setPosition(mAngle);
        mServoPosition.setPosition(mPositionAngle);
        mServoRotation.setPosition(mRotationAngle);
    }

    public void openPince() {
        mAngle = Constants.ConstantsPince.kPinceOpenAngle;
        mPositionAngle = Constants.ConstantsPince.kPinceOpenPosAngle;
    }

    public void closePince() {
        //mServoPince.setAngle(0);
        mAngle = Constants.ConstantsPince.kPinceCloseAngle;
        mPositionAngle = Constants.ConstantsPince.kPinceClosePosAngle;
    }

    public void setAngle(double angle) {
        mAngle += angle;
    }

    public void setPositionAngle(double position) {
        mPositionAngle += position;
    }

    public void setRotationAngle(double rotation) {
        mRotationAngle += rotation;
    }


}


