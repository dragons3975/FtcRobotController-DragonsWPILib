package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import dragons.rev.FtcMotorSimple;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotor m_frontLeftMotor = new FtcMotor("fleft");
    private final FtcMotor m_frontRightMotor = new FtcMotor("fright");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");
    private final FtcMotor m_rearRightMotor = new FtcMotor("rright");
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor,m_rearLeftMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro();
    private double mAngle = 0;

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    private double total = 0;

    private boolean pidActif = true;

    private double angleConsigne = 0;

    private PIDController mPIDz = new PIDController(Constants.ConstantsDrivePID.kP, Constants.ConstantsDrivePID.kI, Constants.ConstantsDrivePID.kD);
    public DriveSubsystem() {
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);

        activerPid();
    }

    @Override
    public void periodic() {

        mAngle = mGyro.getAngle();
        DriverStationJNI.getTelemetry().addData("mGyro angle", mAngle);

        if (pidActif) {
            m_zRotation = mPIDz.calculate(mAngle, angleConsigne);
            DriverStationJNI.getTelemetry().addData("m_zRotation", m_zRotation);
        }

        // On inverse volontairement x et y pour avoir le x vers l'avant
        m_robotDrive.driveCartesian(m_ySpeed, m_xSpeed, m_zRotation);

        DriverStationJNI.getTelemetry().addData("y", getY());
        DriverStationJNI.getTelemetry().addData("x", getX());
        DriverStationJNI.getTelemetry().addData("isAtSetPoint", isAtSetPoint());

        DriverStationJNI.getTelemetry().addData("encodeur Front Left", m_frontLeftMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Front Right", m_frontRightMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Rear Left", m_rearLeftMotor.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("encodeur Rear Right", m_rearRightMotor.getCurrentPosition());
    }

    public double getY() {
        return ((-m_frontRightMotor.getCurrentPosition() -m_rearLeftMotor.getCurrentPosition()) - (m_frontLeftMotor.getCurrentPosition() + m_rearRightMotor.getCurrentPosition())) / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public double getX() {
        return -((-m_frontRightMotor.getCurrentPosition() - m_rearLeftMotor.getCurrentPosition()) + (m_frontLeftMotor.getCurrentPosition() + m_rearRightMotor.getCurrentPosition())) / 4.0 / Constants.ConstantsDrive.distanceCalcul;
    }

    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        m_xSpeed = -xSpeed;
        m_ySpeed = -ySpeed;
        //m_zRotation = zRotation;
        angleConsigne+=zRotation;
    }
    public void getFrontLeft() {
        //m_frontLeftMotor.getCurrentPosition();

    }

    public void activerPid() {
        pidActif = true;
        angleConsigne = mGyro.getAngle();
    }

    public void desactiverPid() {
        pidActif = false;
    }

    public boolean isAtSetPoint() {
        return mPIDz.atSetpoint();
    }

    public void stop () {
        m_xSpeed = 0;
        m_ySpeed = 0;
        m_zRotation = 0;
    }
}



