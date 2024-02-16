
package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PinceSubsystem extends Subsystem {

    private final FtcServo mPince1 = new FtcServo("pince1");
    private final FtcServo mPince2 = new FtcServo("pince2");

    public PinceSubsystem() {
        ouvre();
    }

    public void ouvre() {
        mPince1.setPosition(0.3);
        mPince2.setPosition(0);-
    }
    public void ferme() {
        mPince1.setPosition(0);
        mPince2.setPosition(0.3);
    }



}



