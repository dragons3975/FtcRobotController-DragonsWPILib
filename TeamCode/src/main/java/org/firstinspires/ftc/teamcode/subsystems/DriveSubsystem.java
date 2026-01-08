package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class DriveSubsystem extends Subsystem {

    private final FtcMotor m_frontLeftMotor = new FtcMotor("fleft");
    private final FtcMotor m_frontRightMotor = new FtcMotor("fright");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");
    private final FtcMotor m_rearRightMotor = new FtcMotor("rright");
    //private final FtcGyro m_gyro = new FtcGyro(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_frontRightMotor, m_rearLeftMotor, m_rearRightMotor);
    private final PIDController mPIDx = new PIDController(Constants.DriveConstants.kPx, 0, 0);
    private final PIDController mPIDy = new PIDController(Constants.DriveConstants.kPy, 0, 0);
    private double mYConsigne;
    private double mXConsigne;

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;
    private boolean mIsPIDActivated = false;
    public DriveSubsystem() {
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(true);
        m_rearLeftMotor.setInverted(false);
        m_rearRightMotor.setInverted(false);
        m_robotDrive.setMaxOutput(0.7);

        mPIDy.setTolerance(20);
        mPIDx.setTolerance(20);
    }

    @Override
    public void periodic() {
        if (mIsPIDActivated ){
            m_ySpeed = mPIDy.calculate(getDistanceY(),mYConsigne);
            m_xSpeed = mPIDx.calculate(getDistanceX(),mXConsigne);
        }
        DriverStationJNI.getTelemetry().addData("m_xSpeed", m_xSpeed);
        DriverStationJNI.getTelemetry().addData("m_ySpeed", m_ySpeed);
        DriverStationJNI.getTelemetry().addData("m_zRotation", m_zRotation);
        DriverStationJNI.getTelemetry().addData("distanceY", getDistanceY());
        DriverStationJNI.getTelemetry().addData("distanceX", getDistanceX());
        DriverStationJNI.getTelemetry().addData("consigneY", mYConsigne);
        DriverStationJNI.getTelemetry().addData("consigneX", mXConsigne);


        m_robotDrive.driveCartesian(m_xSpeed, m_ySpeed, m_zRotation);



    }

    public void drive(double x, double y, double z) {
        m_xSpeed = x;
        m_ySpeed = y;
        m_zRotation = z;
    }

    public void stop() {
        drive(0,0,0);
        //mIsPIDActivated = false;
    }
    public double getDistanceY(){
        return (m_frontRightMotor.getCurrentPosition()+m_rearRightMotor.getCurrentPosition())/2;

    }
    public double getDistanceX(){
        return (m_frontRightMotor.getCurrentPosition()-m_rearRightMotor.getCurrentPosition())/2;
    }

    public double getAngle() {
        return 0;//m_gyro.getAngle();
    }
    public void setConsigneY(double consigneY){
        mYConsigne = consigneY;
        mIsPIDActivated = true;
    }
    public void setConsigneX(double consigneX){
        mXConsigne = consigneX;
        mIsPIDActivated = true;
    }

    public boolean isAtSetPointY (){
        return mPIDy.atSetpoint();
    }
    public boolean isAtSetPointX (){
        return mPIDx.atSetpoint();
    }
}