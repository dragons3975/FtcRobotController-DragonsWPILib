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

    private double mAngleConsigne;

    double Delta_middle_pos, prev_center_encodeur_pos, Delta_left_encodeur_pos, Delta_right_encodeur_pos, prev_left_encodeur_pos, prev_right_encodeur_pos, Delta_perp_pos, Delta_center_encodeur_pos;


    double forward_offset = 16; //en cm
    double trackwith = 32;

    double heading; //teta 0
    double phi = 0;
    double phi_degre;
    double Y, X;
    double YApril, XApril;

    double deltaX, deltaY;

    double current_left_encodeur_pos, current_right_encodeur_pos, current_center_encodeur_pos;

    private PIDController mPIDz = new PIDController(Constants.ConstantsDrivePID.kP, Constants.ConstantsDrivePID.kI, Constants.ConstantsDrivePID.kD);
    private VisionSubsystem mVisionSubsystem;

    public DriveSubsystem(VisionSubsystem visionSubsystem) {
        mGyro.reset();
        mVisionSubsystem = visionSubsystem;
        m_robotDrive.setMaxOutput(Constants.ConstantsDrive.kVitesseHaute);
        mPIDz.setTolerance(Constants.ConstantsDrivePID.kToleranceZ);
        m_frontLeftMotor.setInverted(true);
        m_frontRightMotor.setInverted(false);
        m_rearLeftMotor.setInverted(true);
        m_rearRightMotor.setInverted(false);

        mAngleConsigne = getAngle();

        mPIDz.enableContinuousInput(-180, 180);
    }

    @Override
    public void periodic() {
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

        DriverStationJNI.getTelemetry().addData("heading", heading);
        DriverStationJNI.getTelemetry().addData("consigne drive", mAngleConsigne);
        DriverStationJNI.getTelemetry().addData("Y", Y);
        DriverStationJNI.getTelemetry().addData("X", X);
        DriverStationJNI.getTelemetry().addData("YApril", YApril);
        DriverStationJNI.getTelemetry().addData("XApril", XApril);
        DriverStationJNI.getTelemetry().addData("gat angle", getAngle());

        mAngle = getAngle();
        //DriverStationJNI.getTelemetry().addData("mGyro angle", mAngle);
        //DriverStationJNI.getTelemetry().addData("angleConsigne", mAngleConsigne);
        m_zRotation = mPIDz.calculate(mAngle, mAngleConsigne);
        if (Math.abs(m_zRotation) > Constants.MaxSpeeds.kmaxZspeed) {
            m_zRotation = Math.signum(m_zRotation) * Constants.MaxSpeeds.kmaxZspeed;
        }
        //DriverStationJNI.getTelemetry().addData("m_xSpeed", m_xSpeed);
        //DriverStationJNI.getTelemetry().addData("m_ySpeed", m_ySpeed);
        //DriverStationJNI.getTelemetry().addData("m_zRotation", m_zRotation);

        // On inverse volontairement x et y pour avoir le x vers l'avant
        m_robotDrive.driveCartesian(m_ySpeed, m_xSpeed, m_zRotation);

        //DriverStationJNI.getTelemetry().addData("y", getY());
        //DriverStationJNI.getTelemetry().addData("x", getX());
        //DriverStationJNI.getTelemetry().addData("isAtSetPointz", isAtSetPointz());

        DriverStationJNI.getTelemetry().addData("center encodeur", m_frontRightMotor.getCurrentPosition());


    }

    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        mAngleConsigne += zRotation;

        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
    }

    public void testMotor() {
        m_rearRightMotor.set(1);
    }

    public double get_left_encoder_pos() {
        return m_frontLeftMotor.getCurrentPosition() / Constants.ConstantsDrive.ktachoParCm;
    }

    public double get_right_encoder_pos() {
        return -m_frontRightMotor.getCurrentPosition() / Constants.ConstantsDrive.ktachoParCm;
    }

    public double get_center_encoder_pos() {
        return m_rearLeftMotor.getCurrentPosition() / Constants.ConstantsDrive.ktachoParCm;
    }

    public Pose2d getCurrentPose() {
        return new Pose2d(X, Y, Rotation2d.fromDegrees(heading));
    }
    public boolean isAtSetPointz() {
        return mPIDz.atSetpoint();
    }

    public void stop () {
        mAngleConsigne = getAngle();
    }

    private double getAngle() {
        return MathUtil.inputModulus(-mGyro.getAngle(), -180, 180);
    }

}



