package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo leftServo = new FtcServo("leftServo");
    private final FtcServo rightServo = new FtcServo("rightServo");
    private final FtcServo pivotServo = new FtcServo("pivotServo");

    private double mPosition = -0.70;


    public PinceSubsystem() {

        //stopRotation();
    }

    @Override
    public void periodic() {
        pivotServo.setPosition(mPosition);
        DriverStationJNI.getTelemetry().addData("servo pos", pivotServo.getPosition());
    }

    public void IncrementPivotPosition(double increment) {
        mPosition += increment;
        if (mPosition > 0.70) {
            mPosition = 0.70;
        }
        if (mPosition < 0) {
            mPosition = 0;
        }
    }

    public void setGripClosePosition() {
        leftServo.setPosition(Constants.ConstantsPince.kPinceGaucheOuvreMax);
        rightServo.setPosition(Constants.ConstantsPince.kPinceDroitOuvreMax);
    }

    public void setGripOpenPosition() {
        leftServo.setPosition(Constants.ConstantsPince.kPinceGaucheOuvreMin);
        rightServo.setPosition(Constants.ConstantsPince.kPinceDroitOuvreMin);
    }

}



