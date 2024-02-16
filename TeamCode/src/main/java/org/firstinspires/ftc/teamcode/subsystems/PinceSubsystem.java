
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPince = new FtcServo("pince test");

    //private final FtcServo mMotorPince = new FtcServo("pinceRotation");

    //private final FtcServo mMotorPinceG = new FtcServo("pinceGauche");

    //private final FtcServo mMotorPinceD = new FtcServo("pinceDroite");
    private boolean mOuverte = false;

    private double pos = 0;
    public PinceSubsystem() {
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("etat pince", mOuverte);
        DriverStationJNI.getTelemetry().addData("etat moteur", mMotorPince.getPosition());
    }

    //public void ModifInclinaison(double pos) {
    //    mMotorPince.setPosition(pos);
    //}


    public void Ferme() {
        mMotorPince.setPosition(Constants.ConstantsPince.ouvreMin);
        mOuverte = false;
    }

    public void Ouvre() {
        mMotorPince.setPosition(Constants.ConstantsPince.ouvreMax);
        mOuverte = true;
    }

    public void Toggle(){
        if (mOuverte){
            Ferme();
        }
        else {
            Ouvre();
        }
    }

}



