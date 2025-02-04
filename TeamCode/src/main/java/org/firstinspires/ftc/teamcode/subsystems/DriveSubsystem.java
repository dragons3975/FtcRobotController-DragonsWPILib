package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class DriveSubsystem extends Subsystem {
    private final FtcMotor m_frontLeftMotor = new FtcMotor("fleft");
    private final FtcMotor m_frontRightMotor = new FtcMotor("fright");
    private final FtcMotor m_rearLeftMotor = new FtcMotor("rleft");
    private final FtcMotor m_rearRightMotor = new FtcMotor("rright");
    private final MecanumDrive m_robotDrive = new MecanumDrive(m_frontLeftMotor, m_rearLeftMotor, m_frontRightMotor, m_rearRightMotor);
    private final FtcGyro mGyro = new FtcGyro(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
    private double mAngle = 0;

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    private double m_xAuto, m_yAuto;

    private double mAngleConsigne;

    double Delta_middle_pos, prev_center_encodeur_pos, Delta_left_encodeur_pos, Delta_right_encodeur_pos, prev_left_encodeur_pos, prev_right_encodeur_pos, Delta_perp_pos, Delta_center_encodeur_pos;


    double forward_offset = 16; //en cm
    double trackwith = 32;

    double heading; //teta 0
    double phi = 0;
    double phi_degre;
    double Y, X;
    double YApril, XApril;

    boolean isAutonomous = false;

    double deltaX, deltaY;

    double current_left_encodeur_pos, current_right_encodeur_pos, current_center_encodeur_pos;

    private PIDController mPIDz = new PIDController(Constants.ConstantsDrivePID.kP, Constants.ConstantsDrivePID.kI, Constants.ConstantsDrivePID.kD);
    private PIDController mPIDxAuto = new PIDController(0.03, 0, 0);
    private PIDController mPIDyAuto = new PIDController(0.03, 0, 0);
    private VisionSubsystem mVisionSubsystem;

    public DriveSubsystem(VisionSubsystem visionSubsystem) {
        mVisionSubsystem = visionSubsystem;
        m_robotDrive.setMaxOutput(Constants.ConstantsDrive.kVitesseHaute);
        mPIDz.setTolerance(Constants.ConstantsDrivePID.kToleranceZ);
        mPIDxAuto.setTolerance(2);
        mPIDyAuto.setTolerance(2);
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(false);
        m_rearLeftMotor.setInverted(true);
        m_rearRightMotor.setInverted(false);

        mAngleConsigne = getAngle();

        mPIDz.enableContinuousInput(-180, 180);
    }

    @Override
    public void periodic() {
        ///////////////
        if (mVisionSubsystem.isDetecting()) {
            XApril = mVisionSubsystem.getPosition().getX() * 2.54;
            YApril = mVisionSubsystem.getPosition().getY() * 2.54;
        }

        current_left_encodeur_pos = get_left_encoder_pos();
        current_right_encodeur_pos = get_right_encoder_pos();
        current_center_encodeur_pos = get_center_encoder_pos();

        Delta_left_encodeur_pos = current_left_encodeur_pos - prev_left_encodeur_pos;
        Delta_right_encodeur_pos = current_right_encodeur_pos - prev_right_encodeur_pos;
        Delta_center_encodeur_pos = current_center_encodeur_pos - prev_center_encodeur_pos;

        phi = (Delta_left_encodeur_pos - Delta_right_encodeur_pos) / trackwith * 0.8; //en radian
        phi_degre = Math.toDegrees(phi);
        Delta_perp_pos = Delta_center_encodeur_pos - forward_offset * phi;
        Delta_middle_pos = (Delta_left_encodeur_pos + Delta_right_encodeur_pos) / 2;

        heading -= phi_degre;

        deltaX = Delta_middle_pos * Math.cos(Math.toRadians(heading)) - Delta_perp_pos * Math.sin(Math.toRadians(heading));
        deltaY = Delta_middle_pos * Math.sin(Math.toRadians(heading)) + Delta_perp_pos * Math.cos(Math.toRadians(heading));

        Y += deltaY * 0.778;
        X += deltaX * 0.796;

        prev_left_encodeur_pos = current_left_encodeur_pos;
        prev_right_encodeur_pos = current_right_encodeur_pos;
        prev_center_encodeur_pos = current_center_encodeur_pos;
        //////////////////////////////

        if (isAutonomous) {
            m_xSpeed = mPIDxAuto.calculate(X, m_xAuto);
            m_ySpeed = mPIDyAuto.calculate(Y, m_yAuto);
        }


        DriverStationJNI.getTelemetry().addData("heading", heading);
        DriverStationJNI.getTelemetry().addData("consigne drive", mAngleConsigne);
        DriverStationJNI.getTelemetry().addData("Y", Y);
        DriverStationJNI.getTelemetry().addData("X", X);
        DriverStationJNI.getTelemetry().addData("xSpeed", m_xSpeed);
        DriverStationJNI.getTelemetry().addData("ySpeed", m_ySpeed);
        DriverStationJNI.getTelemetry().addData("gat angle", getAngle());

        mAngle = getAngle();
        m_zRotation = mPIDz.calculate(mAngle, mAngleConsigne);
        if (Math.abs(m_zRotation) > Constants.MaxSpeeds.kmaxZspeed) {
            m_zRotation = Math.signum(m_zRotation) * Constants.MaxSpeeds.kmaxZspeed;
        }

        m_robotDrive.driveCartesian(m_xSpeed, m_ySpeed, m_zRotation);

        DriverStationJNI.getTelemetry().addData("center encodeur", get_center_encoder_pos());
        DriverStationJNI.getTelemetry().addData("gauche encodeur", get_left_encoder_pos());
        DriverStationJNI.getTelemetry().addData("droite encodeur", get_right_encoder_pos());


    }

    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
        mAngleConsigne += zRotation;
        isAutonomous = false;
    }

    public void mecanumDrivePID(double distX, double distY) {
        mAngleConsigne = mAngle;
        m_xAuto = distX;
        m_yAuto = distY;
        isAutonomous = true;
    }

    public void testMotor() {
        m_rearRightMotor.set(1);
    }

    public double get_left_encoder_pos() {
        return m_frontLeftMotor.getCurrentPosition() / -Constants.ConstantsDrive.ktachoParCm;
    }

    public double get_right_encoder_pos() {
        return -m_frontRightMotor.getCurrentPosition() / Constants.ConstantsDrive.ktachoParCm;
    }

    public double get_center_encoder_pos() {
        return m_rearLeftMotor.getCurrentPosition() / -Constants.ConstantsDrive.ktachoParCm;
    }

    public Pose2d getCurrentPose() {
        return new Pose2d(X, Y, Rotation2d.fromDegrees(heading));
    }
    public boolean isAtSetPointz() {
        return mPIDz.atSetpoint();
    }
    public boolean isAtSetPointx() {
        return mPIDxAuto.atSetpoint();
    }
    public boolean isAtSetPointy() {
        return mPIDyAuto.atSetpoint();
    }

    public void stop () {
        isAutonomous = false;
        mAngleConsigne = getAngle();
        m_frontLeftMotor.stopMotor();
        m_frontRightMotor.stopMotor();
        m_rearLeftMotor.stopMotor();
        m_rearRightMotor.stopMotor();
    }

    private double getAngle() {
        return MathUtil.inputModulus(-mGyro.getAngle(), -180, 180);
    }

    public void resetGyro() {
        mGyro.reset();
    }

    public void resetDeplacement() {
        isAutonomous = false;
        mGyro.reset();
        X = 0;
        Y = 0;
    }

}



