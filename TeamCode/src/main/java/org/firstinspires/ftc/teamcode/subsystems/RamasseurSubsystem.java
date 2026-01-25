package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class RamasseurSubsystem extends Subsystem {

    private final FtcMotor mMotorRamasseur = new FtcMotor("ramasseur");

    public RamasseurSubsystem() {
        mMotorRamasseur.setInverted(true);
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Motor lanceur encodeur", mMotorRamasseur.getCurrentPosition());
    }

    public void ramasse() {
        mMotorRamasseur.set(1);
    }
    public void rejette() {
        mMotorRamasseur.set(-1);
    }

    public void stop() {
        mMotorRamasseur.stopMotor();
    }

}