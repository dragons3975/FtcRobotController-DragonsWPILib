package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceBrasSubsystem extends Subsystem {

    private final FtcServo mServoPince = new FtcServo("PinceBras");

    private final FtcServo mServoPincePosition = new FtcServo("PincePositionBras");

    private double mAngle = 0;
    private double mPositionAngle = 0;

    public PinceBrasSubsystem() {
    }

    @Override
    public void periodic() {
        DriverStationJNI.Telemetry.putNumber("Servo Pince Angle", mAngle);
        DriverStationJNI.Telemetry.putNumber("Servo Pince Position", mPositionAngle);
        mServoPince.setPosition(mAngle);
        mServoPincePosition.setPosition(mPositionAngle);
    }

    public void openPince() {
        mAngle = Constants.ConstantsPince.kPinceOpenAngle;
    }

    public void closePince() {
        //mServoPince.setAngle(0);
        mAngle = Constants.ConstantsPince.kPinceCloseAngle;
    }

    public void setAngle(double angle) {
        mAngle = angle;
    }

    public void setPositionAngle(double position) {
        mPositionAngle = position;
    }

}