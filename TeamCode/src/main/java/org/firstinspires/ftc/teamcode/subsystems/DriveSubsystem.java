package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcMotor;
import dragons.rev.HolonomicDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    public final FtcMotor m_roueright = new FtcMotor("roue1");
    public final FtcMotor m_roueleft = new FtcMotor("roue2");
    public final FtcMotor m_roueback = new FtcMotor("roue3");

    private final HolonomicDrive m_robotdrive = new HolonomicDrive(m_roueleft, m_roueright, m_roueback);

    private double mX;
    private double mY;
    private double mZ;

    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
        m_robotdrive.holonomicDrive(mX, mY, mZ);
    }

    public void drive(double x, double y, double z) {
        mX = x;
        mY = y;
        mZ = z;
    }

    public void stop(){
        drive(0, 0, 0);
    }




}



