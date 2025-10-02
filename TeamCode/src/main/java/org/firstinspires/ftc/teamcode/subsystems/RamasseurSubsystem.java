package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class RamasseurSubsystem extends Subsystem {

    private final FtcMotor mMotorTest3 = new FtcMotor("ramasseur");

    public RamasseurSubsystem() {
    }

    @Override
    public void periodic() {

    }

    public void start() {
        mMotorTest3.set(0.5);
    }
    public void pain() {
        mMotorTest3.set(-0.5);
    }

    public void stop() {
        mMotorTest3.stopMotor();
    }

}