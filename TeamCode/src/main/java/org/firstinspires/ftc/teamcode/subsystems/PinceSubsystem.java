
package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPinceInclinaison = new FtcServo("pince inclinaison");
    private final FtcServo mMotorPinceDroit = new FtcServo("pince droit");
    private final FtcServo mMotorPinceGauche = new FtcServo("pince gauche");

    private boolean gaucheOuvert = false;
    private boolean droiteOuvert = false;
    private boolean mOuverte = false;
    private boolean bas = false;

    public PinceSubsystem() {
        InclineBas();
        Ferme();
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("etat pince", mOuverte);
        DriverStationJNI.getTelemetry().addData("etat inclinaison", mMotorPinceInclinaison.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo gauche", mMotorPinceGauche.getPosition());
        DriverStationJNI.getTelemetry().addData("etat servo droit", mMotorPinceDroit.getPosition());
    }

    public void OuvreGauche() {
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.kPinceGaucheOuvreMax);
        gaucheOuvert = true;
    }

    public void FermeGauche() {
        mMotorPinceGauche.setPosition(Constants.ConstantsPince.kPinceGaucheOuvreMin);
        gaucheOuvert = false;
    }

    public void ToggleGauche() {
        if (gaucheOuvert) {
            FermeGauche();
        } else {
            OuvreGauche();
        }
    }

    public void OuvreDroit() {
        mMotorPinceDroit.setPosition(Constants.ConstantsPince.kPinceDroitOuvreMax);
        droiteOuvert = true;
    }

    public void FermeDroit() {
        mMotorPinceDroit.setPosition(Constants.ConstantsPince.kPinceDroitOuvreMin);
        droiteOuvert = false;
    }

    public void ToggleDroite() {
        if (droiteOuvert) {
            FermeDroit();
        } else {
            OuvreDroit();
        }
    }

    public void Ouvre() {
        OuvreGauche();
        OuvreDroit();
        mOuverte = true;
    }

    public void Ferme() {
        FermeGauche();
        FermeDroit();
        mOuverte = false;
    }

    public void Toggle(){
        if (mOuverte){
            Ferme();
        }
        else {
            Ouvre();
        }
    }


    public void InclineBas() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.kInclinaisonBas);
        bas = true;
    }

    public void InclineHaut() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.kInclinaisonHaut);
        bas = false;
    }

    public void InclinePile() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.kInclinaisonPile);
    }

    public void InclineToggle() {
        if (bas) {
            InclineHaut();
        } else {
            InclineBas();
        }
    }

    public void InclinaisonSolSecurite() {
        mMotorPinceInclinaison.setPosition(Constants.ConstantsPince.kInclinaisonMinSol);
        bas = true;
    }

}
