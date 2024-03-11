
package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BoucheSubsystem extends Subsystem {

    private final FtcMotor mMotorBouche = new FtcMotor("exMotor3");
    public double power = 0;


    public BoucheSubsystem() {
        mMotorBouche.brakeOnZeroPower(false);
    }

    @Override
    public void periodic() {
        mMotorBouche.setVoltage(power);
    }
}



