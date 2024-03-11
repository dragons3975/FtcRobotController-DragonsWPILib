package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotor m_frontLeftMotor = new FtcMotor("motor2");
    private final FtcMotor m_frontRightMotor = new FtcMotor("exMotor0");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("motor3");
    private final FtcMotor m_rearRightMotor = new FtcMotor("exMotor1");
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor ,m_rearLeftMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro();
    private double mAngle = 0;
    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    public DriveSubsystem() {
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);

        m_frontRightMotor.brakeOnZeroPower(true);
        m_frontLeftMotor.brakeOnZeroPower(true);
        m_rearRightMotor.brakeOnZeroPower(true);
        m_rearLeftMotor.brakeOnZeroPower(true);
    }

    @Override
    public void periodic() {

        mAngle = mGyro.getAngle();
        DriverStationJNI.getTelemetry().addData("mGyro angle", mAngle);

        m_robotDrive.driveCartesian(m_xSpeed, m_ySpeed, m_zRotation);

        DriverStationJNI.getTelemetry().addData("y", getY());
        DriverStationJNI.getTelemetry().addData("x", getX());

        DriverStationJNI.getTelemetry().addData("encodeur Front Left", m_frontLeftMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Front Right", m_frontRightMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Rear Left", m_rearLeftMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Rear Right", m_rearRightMotor.getCurrentPosition());
    }

    public double getY() {
        return 0;//((-m_frontRightMotor.getCurrentPosition() -m_rearLeftMotor.getCurrentPosition()) - (m_frontLeftMotor.getCurrentPosition() + m_rearRightMotor.getCurrentPosition())) / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public double getX() {
        return 0;//-((-m_frontRightMotor.getCurrentPosition() - m_rearLeftMotor.getCurrentPosition()) + (m_frontLeftMotor.getCurrentPosition() + m_rearRightMotor.getCurrentPosition())) / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        m_zRotation = zRotation;
    }

    public void stop () {
        m_xSpeed = 0;
        m_ySpeed = 0;
        m_zRotation = 0;
    }
}



