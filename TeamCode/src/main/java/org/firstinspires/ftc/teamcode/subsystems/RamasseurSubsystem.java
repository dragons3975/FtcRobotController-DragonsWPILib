package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class RamasseurSubsystem extends Subsystem {

    private final FtcMotor mMotorTest3 = new FtcMotor("ramasseurtest");

    public RamasseurSubsystem() {
    }

    @Override
    public void periodic() {

    }

    public void monte() {
        mMotorTest3.set(1);
    }
    public void descend() {
        mMotorTest3.set(-1);
    }

    public void stop() {
        mMotorTest3.stopMotor();
    }

}