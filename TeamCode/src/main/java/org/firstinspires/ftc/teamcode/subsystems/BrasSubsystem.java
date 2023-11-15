package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcMotor m_MotorBras = new FtcMotor("bras");
    private final PIDController m_zPID = new PIDController(0.02, 0, 0);

    private double m_posTarget = 0;

    public BrasSubsystem() {
        m_zPID.setTolerance(3);
    }

    @Override
    public void periodic() {
        double consigne = m_zPID.calculate(m_MotorBras.getCurrentPosition(), m_posTarget);
        DriverStationJNI.getTelemetry().addData("currentPosition", m_MotorBras.getCurrentPosition());
        m_MotorBras.set(consigne);
    }

    public void setTarget(double target) {
        m_posTarget = target;
    }

}



