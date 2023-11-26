package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import dragons.rev.FtcMotor;
import dragons.rev.FtcMotorSimple;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotorSimple m_frontLeftMotor = new FtcMotorSimple("fleft");
    private final FtcMotorSimple m_frontRightMotor = new FtcMotorSimple("fright");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");
    private final FtcMotor m_rearRightMotor = new FtcMotor("rright");
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor,m_rearLeftMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro();
private double mAngle = 0;

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    private PIDController mPIDz = new PIDController(Constants.ConstantsDrivePID.kP,Constants.ConstantsDrivePID.kI, Constants.ConstantsDrivePID.kD);
    public DriveSubsystem() {
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);

    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("distanceX", getDistanceX());
        m_robotDrive.driveCartesian(m_xSpeed, m_zRotation, m_ySpeed);
        mAngle = mGyro.getAngle();
        DriverStationJNI.getTelemetry().addData("mGyro angle", mAngle);
    }


    public double getDistanceX() {
        return m_rearLeftMotor.getCurrentPosition() / (8192/(Math.PI * 5.08));
    }

    public void mecanumDrive(double xSpeed, double zRotation, double ySpeed){
        m_xSpeed = xSpeed;
        m_zRotation = zRotation;
        m_ySpeed = ySpeed;
    }
    public void getFrontLeft() {
        //m_frontLeftMotor.getCurrentPosition();

    }

    public void stop () {
        m_xSpeed = 0;
        m_ySpeed = 0;
        m_zRotation = 0;
    }
}



