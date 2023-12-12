
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mMotorPince = new FtcServo("pince");
    private  final FtcCRServo mMoteurPinceTourne = new FtcCRServo("pincetourne");
    private boolean mOuverte = true;

    public PinceSubsystem() {
    }

    public void Test(double degre) {
        mMoteurPinceTourne.setInverted(true);
        mMoteurPinceTourne.set(-1);
    }


    public void Ouvrir() {
        //mMotorPince.setposition (0.5,ouvert);
        mOuverte = true;

    }

    public void Fermer() {
        //mMotorPince.setPosition(-0.5, fermer);
        mOuverte = false;
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



