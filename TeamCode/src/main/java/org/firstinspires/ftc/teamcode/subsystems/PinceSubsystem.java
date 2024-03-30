
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPinceInclinaison = new FtcServo("pince inclinaison");
    private final FtcServo mMotorPinceDroit = new FtcServo("pince droit");
    private final FtcServo mMotorPinceGauche = new FtcServo("pince gauche");

    private boolean bas = true;

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
        DriverStationJNI.getTelemetry().addData("etat servo inclinaison", mMotorPinceInclinaison.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo gauche", mMotorPinceGauche.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo droit", mMotorPinceDroit.getPosition());
    }



    public void Ferme() {
        mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMin);
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMax);
        mOuverte = false;
    }

    public void Ouvre() {
        mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMax);
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMin);
        mOuverte = true;
    }

    public void InclineToggle() {
        if (bas) {
            mMotorPinceInclinaison.setPosition(0);
            bas = false;
        } else {
            mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.Inclinaison);
            bas = true;
        }
    }

    public void InclineBas() {
        mMotorPinceInclinaison.setPosition(0);
    }

    public void InclineHaut() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.Inclinaison);
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



