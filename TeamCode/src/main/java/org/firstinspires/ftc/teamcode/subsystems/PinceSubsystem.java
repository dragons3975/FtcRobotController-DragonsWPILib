
package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mServoPince = new FtcServo("servo0");
    private boolean mOuverte = false;

    private double pos = 0.5;
    public PinceSubsystem() {
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("etat pince", mOuverte);
        mServoPince.setPosition(pos);
    }

    public void ChangeState(){
       mOuverte = !mOuverte;
       if (mOuverte){
           pos=0.58;
       }
       else {
           pos=0.51;
       }
    }
}



