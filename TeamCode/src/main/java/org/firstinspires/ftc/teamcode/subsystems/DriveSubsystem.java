package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcGyro;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    //private final FtcMotorSimple m_frontLeftMotor = new FtcMotorSimple("fleft");
    //private final FtcMotorSimple m_frontRightMotor = new FtcMotorSimple("fright");
    //private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");/
    //private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor,m_rearLeftMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro();

    public DriveSubsystem() {
    }

    @Override
    public void periodic() {
    }


}



