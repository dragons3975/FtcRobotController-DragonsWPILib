
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcCRServo;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class GrimpeurSubsystem extends Subsystem {

    private final FtcServo mMotorLeft = new FtcServo("grimpeur left");

    private final FtcServo mMotorRight = new FtcServo("grimpeur right");

    public GrimpeurSubsystem() {
    }

    @Override
    public void periodic() {
    }

    //public void ModifInclinaison(double pos) {
    //    mMotorPince.setPosition(pos);
    //}


    public void Ferme() {
        mMotorLeft.setPosition(0);
        mMotorRight.setPosition(0);
    }

    public void Ouvre() {
        mMotorLeft.setPosition(0.5);
        mMotorRight.setPosition(0.5);
    }

}



