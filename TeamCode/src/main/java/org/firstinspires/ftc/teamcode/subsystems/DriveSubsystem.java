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

    private double m_xSpeed = 0; // The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
    private double m_zRotation = 0; // The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
    private double m_ySpeed = 0;

    private double m_trackwidth = 0;
    private double m_forward_offset = 0;

    private double heading = 0;
    private double m_x, m_y, m_z, m_prev_left_encoder_pos, m_prev_right_encoder_pos, m_prev_center_encoder_pos = 0;
    public DriveSubsystem() {
        m_robotDrive.setMaxOutput(Constants.ConstantsDrive.kVitesseHaute);
        m_frontLeftMotor.setInverted(false);
        m_frontRightMotor.setInverted(false);
        m_rearLeftMotor.setInverted(true);
        m_rearRightMotor.setInverted(true);

    }

    @Override
    public void periodic() {
        double delta_left_encodeur_pos = get_left_encoder_pos() - m_prev_left_encoder_pos;
        double delta_right_encodeur_pos = get_right_encoder_pos() - m_prev_right_encoder_pos;
        double delta_center_encodeur_pos = get_center_encoder_pos() - m_prev_center_encoder_pos;

        double phi = (delta_left_encodeur_pos - delta_right_encodeur_pos) / m_trackwidth;
        double delta_middle_pos = (delta_left_encodeur_pos + delta_right_encodeur_pos) / 2;
        double delta_perp_pos = delta_center_encodeur_pos - m_forward_offset * phi;

        double heading_rad = Math.toRadians(heading);
        double delta_x = delta_middle_pos * Math.cos(heading_rad) - delta_perp_pos * Math.sin(heading_rad);
        double delta_y = delta_middle_pos * Math.sin(heading_rad) + delta_perp_pos * Math.cos(heading_rad);
        heading = Math.toDegrees(heading_rad);

        m_x += delta_x;
        m_y += delta_y;
        heading += phi;

        m_prev_left_encoder_pos = get_left_encoder_pos();
        m_prev_right_encoder_pos = get_right_encoder_pos();
        m_prev_center_encoder_pos = get_center_encoder_pos();





        DriverStationJNI.getTelemetry().addData("m_xSpeed", m_xSpeed);
        DriverStationJNI.getTelemetry().addData("m_ySpeed", m_ySpeed);
        DriverStationJNI.getTelemetry().addData("m_zRotation", m_zRotation);

        // On inverse volontairement x et y pour avoir le x vers l'avant
        m_robotDrive.driveCartesian(m_ySpeed, m_xSpeed, m_zRotation);

        DriverStationJNI.getTelemetry().addData("y", getY());
        DriverStationJNI.getTelemetry().addData("x", getX());
    }

    public double get_left_encoder_pos() {
        return m_rearLeftMotor.getCurrentPosition();
    }

    public double get_right_encoder_pos() {
        return m_rearRightMotor.getCurrentPosition();
    }

    public double get_center_encoder_pos() {
        return m_frontRightMotor.getCurrentPosition();
    }

    public double getY() {
    }

    public double getX() {
    }



    public void mecanumDrive(double xSpeed, double ySpeed, double zRotation){
        m_zRotation = zRotation;
        m_xSpeed = xSpeed;
        m_ySpeed = ySpeed;
    }

    public void stop () {
        m_xSpeed = 0;
        m_ySpeed = 0;
        m_zRotation = 0;
    }
}



