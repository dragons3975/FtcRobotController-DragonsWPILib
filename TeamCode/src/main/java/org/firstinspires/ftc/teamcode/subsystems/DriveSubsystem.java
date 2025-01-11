package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import dragons.rev.FtcMotorSimple;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotor m_frontLeftMotor = new FtcMotor("fleft");
    private final FtcMotor m_frontRightMotor = new FtcMotor("fright");
    private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    private final FtcGyro mGyro = new FtcGyro(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.


    public DriveSubsystem() {
        m_robotDrive.setMaxOutput(Constants.ConstantsDrive.kVitesseHaute);
        m_frontLeftMotor.setInverted(false);
        m_frontRightMotor.setInverted(false);

    }

    @Override
    public void periodic() {


        m_robotDrive.arcadeDrive(m_zRotation, m_xSpeed);

//        DriverStationJNI.getTelemetry().addData("x", getX());

        DriverStationJNI.getTelemetry().addData("front left", getFrontLeftPosition());
        DriverStationJNI.getTelemetry().addData("front right", getFrontRightPosition());
    }


//    public double getX() {
//        return (getFrontRightPosition() + getRearLeftPosition()
//                + getFrontLeftPosition() + getRearRightPosition())
//                / 4.0 * Constants.ConstantsDrive.distanceCalculx;
//    }

    public void tankDrive(double xSpeed, double zRotation){
        m_xSpeed = xSpeed;
        m_zRotation = zRotation;
    }

    private double getFrontLeftPosition() {
        return m_frontLeftMotor.getCurrentPosition();
    }

    public void stop() {
        m_xSpeed = 0;
        m_zRotation = 0;
    }

        private double getFrontRightPosition() {
        return -m_frontRightMotor.getCurrentPosition();
    }
}



