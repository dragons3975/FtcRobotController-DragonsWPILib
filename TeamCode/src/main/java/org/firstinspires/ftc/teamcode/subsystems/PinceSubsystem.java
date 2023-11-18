
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    //private final Servo mMotorPince = new Servo();
    private boolean mOuverte = true;

    public PinceSubsystem() {
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
            //fermer();
        }
        else {
            //ouvrir();
        }
    }

}



