package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceExtensionSubsystem extends Subsystem {

    private final FtcServo mServoPince = new FtcServo("PinceExt");//2

    private final FtcServo mServoPosition = new FtcServo("PositionPinceExt");//3

    private final FtcServo mServoRotation = new FtcServo("RotationPinceExt");//4

    private double mAngle = 0;
    private double mPositionAngle = 0.5;
    private double mRotationAngle = 0.5;

    public PinceExtensionSubsystem() {
        openPince();
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
    }

    public void closePince() {
        mAngle = Constants.ConstantsPince.kPinceCloseAngle;
    }

    public void setPositionAngle(double position) {
        mPositionAngle = position;
    }

    public void incrementRotationAngle(double rotation) {
        mRotationAngle += rotation;
    }


}


