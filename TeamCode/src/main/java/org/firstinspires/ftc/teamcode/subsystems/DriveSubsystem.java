package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import dragons.rev.FtcMotorSimple;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotor m_frontLeftMotor = new FtcMotor("fleft");
    private final FtcMotor m_frontRightMotor = new FtcMotor("fright");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");
    private final FtcMotor m_rearRightMotor = new FtcMotor("rright");
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor, m_rearLeftMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro();
    private double mAngle = 0;

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    private final boolean orienteTerrain = false;


    private double angleConsigne;

    private final PIDController mPIDz = new PIDController(Constants.ConstantsDrivePID.kP, Constants.ConstantsDrivePID.kI, Constants.ConstantsDrivePID.kD);
    public DriveSubsystem() {
        mPIDz.setTolerance(0);
        m_frontLeftMotor.setInverted(false);
        m_frontRightMotor.setInverted(false);
        m_rearLeftMotor.setInverted(true);
        m_rearRightMotor.setInverted(true);

        angleConsigne = getAngle();

        mPIDz.enableContinuousInput(-180, 180);
    }

    @Override
    public void periodic() {

        mAngle = getAngle();
        DriverStationJNI.getTelemetry().addData("mGyro angle", mAngle);
        DriverStationJNI.getTelemetry().addData("angleConsigne", angleConsigne);
        m_zRotation = mPIDz.calculate(mAngle, angleConsigne);
        if (m_zRotation > 0.2) m_zRotation = 0.2;
        if (m_zRotation < -0.2) m_zRotation = -0.2;
        DriverStationJNI.getTelemetry().addData("m_zRotation", m_zRotation);

        // On inverse volontairement x et y pour avoir le x vers l'avant
        m_robotDrive.driveCartesian(m_ySpeed, m_xSpeed, m_zRotation);

        DriverStationJNI.getTelemetry().addData("y", getY());
        DriverStationJNI.getTelemetry().addData("x", getX());
        DriverStationJNI.getTelemetry().addData("isAtSetPoint", isAtSetPoint());

    }

    public double getY() {
        return (getFrontRightPosition() + getRearLeftPosition()
                - getFrontLeftPosition() - getRearRightPosition())
                / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public double getX() {
        return (getFrontRightPosition() + getRearLeftPosition()
                + getFrontLeftPosition() + getRearRightPosition())
                / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        if (orienteTerrain) {
            angleConsigne += zRotation;
            m_xSpeed = xSpeed * Math.sin(angleConsigne) - xSpeed * Math.cos(angleConsigne);
            m_ySpeed = -xSpeed * Math.cos(angleConsigne) - ySpeed * Math.sin(angleConsigne);
        } else {
            m_xSpeed = xSpeed;
            m_ySpeed = -ySpeed;
            angleConsigne = getAngle() + zRotation;
        }

    }

    public boolean isAtSetPoint() {
        return mPIDz.atSetpoint();
    }

    private double getFrontLeftPosition() {
        return m_frontLeftMotor.getCurrentPosition();
    }
    private double getFrontRightPosition() {
        return -m_frontRightMotor.getCurrentPosition();
    }
    private double getRearLeftPosition() {
        return -m_rearLeftMotor.getCurrentPosition();
    }
    private double getRearRightPosition() {
        return m_rearRightMotor.getCurrentPosition();
    }

    public void stop () {
        m_xSpeed = 0;
        m_ySpeed = 0;
        m_zRotation = 0;
    }

    private double getAngle() {
        return -mGyro.getAngle();
    }
}



