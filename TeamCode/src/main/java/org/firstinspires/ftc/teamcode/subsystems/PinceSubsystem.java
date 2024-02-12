
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPince = new FtcServo("pince test");

    //private final FtcServo mMotorPince = new FtcServo("pinceRotation");

    //private final FtcServo mMotorPinceG = new FtcServo("pinceGauche");

    //private final FtcServo mMotorPinceD = new FtcServo("pinceDroite");
    private boolean mOuverte = true;

    private double pos = 0;

    public PinceSubsystem() {
    }


    //public void ModifInclinaison(double pos) {
    //    mMotorPince.setPosition(pos);
    //}


    public void Ferme() {
        mMotorPince.setPosition(Constants.ConstantsPince.ouvreMin);

    }

    public void Ouvre() {
        mMotorPince.setPosition(Constants.ConstantsPince.ouvreMax);
    }

    public void Toggle(){
        if (mOuverte) {
            mOuverte = false;
            Ferme();
        }
        else {
            mOuverte = true;
            Ouvre();
        }
    }

}



