
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPince = new FtcServo("pinceRotation");

    private final FtcServo mMotorPinceG = new FtcServo("pinceGauche");

    private final FtcServo mMotorPinceD = new FtcServo("pinceDroite");
    private boolean mOuverte = true;

    private double pos = 0;

    public PinceSubsystem() {
    }


    public void ModifInclinaison(double pos) {
        mMotorPince.setPosition(pos);
    }


    public void Ouvrir() {
        //mMotorPince.setposition (0.5,ouvert);
        mOuverte = true;

    }

    public void Fermer() {
        //mMotorPince.setPosition(-0.5, fermer);
        mOuverte = false;
    }

    public void Ferme() {
        mMotorPinceG.setPosition(Constants.ConstantsPince.etenduePince);
        mMotorPinceD.setPosition(0);

    }

    public void Ouvre() {
        mMotorPinceG.setPosition(0);
        mMotorPinceD.setPosition(Constants.ConstantsPince.etenduePince);
    }

    public void Toggle(){
        if (mOuverte) {
            //Fermer();
        }
        else {
            //Ouvrir();
        }
    }

}



