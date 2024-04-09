
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPinceInclinaison = new FtcServo("pince inclinaison");
    private final FtcServo mMotorPinceDroit = new FtcServo("pince droit");
    private final FtcServo mMotorPinceGauche = new FtcServo("pince gauche");

    //private final FtcTouchSensor mTouch = new FtcTouchSensor("touch sensor");

    private boolean gaucheOuvert = false;
    private boolean droiteOuvert = false;

    private boolean bas = true;

    private double max = 0;

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
        DriverStationJNI.getTelemetry().addData("etat inclinaison", mMotorPinceInclinaison.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo gauche", mMotorPinceGauche.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo droit", mMotorPinceDroit.getPosition());
    }

    //public void ModifInclinaison(double pos) {
    //    mMotorPince.setPosition(pos);
    //}

    public void ToggleGauche() {
        if (gaucheOuvert) {
            mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMin);
            gaucheOuvert = false;
        } else {
            mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMax);
            gaucheOuvert = true;
        }
    }

    public void calibrate() {
        mMotorPinceInclinaison.setPosition(max);
    }

    //public boolean isCalibrate() {
        //return mTouch.getState();
    //}

    public void ToggleDroite() {
        if (droiteOuvert) {
            mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMin - 0.05);
            droiteOuvert = false;
        } else {
            mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMax);
            droiteOuvert = true;
        }
    }


    public void Ferme() {
        droiteOuvert = false;
        gaucheOuvert = false;

        mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMin - 0.05);
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMax);
        mOuverte = false;
    }

    public void Ouvre() {
        droiteOuvert = true;
        gaucheOuvert = true;
        mMotorPinceDroit.setPosition(Constants.ConstantsPince.ouvreMax);
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.ouvreMin);
        mOuverte = true;
    }

    public void InclineToggle() {
        if (bas) {
            mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.InclinaisonBas);
            bas = false;
        } else {
            mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.InclinaisonHaut);
            bas = true;
        }
    }

    public void InclineBas() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.InclinaisonBas);
    }

    public void InclineHaut() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.InclinaisonHaut);
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



