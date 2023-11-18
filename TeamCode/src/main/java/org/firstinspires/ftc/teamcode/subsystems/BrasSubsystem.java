package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcMotor;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {

    private final FtcTouchSensor mTouchSensor = new FtcTouchSensor("TouchSensor");

    private final FtcMotor m_MotorBras = new FtcMotor("bras");
    private final PIDController m_zPID = new PIDController(0.001, 0, 0);

    private double m_posTarget = 0;

    private boolean pidActive = false;

    private double init = 0;

    public BrasSubsystem() {
        m_zPID.setTolerance(2);
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("bouttonApuye", verifButton());
        if (pidActive) {
            double consigne = m_zPID.calculate(m_MotorBras.getCurrentPosition(), m_posTarget);
            DriverStationJNI.getTelemetry().addData("currentPosition", m_MotorBras.getCurrentPosition());
            DriverStationJNI.getTelemetry().addData("target", m_posTarget);
            m_MotorBras.set(consigne);
        }
    }


    public void incrementTarget(double target) {
        m_posTarget += target;
    }

    public void setTarget(double target) {
        m_posTarget = init + target;
    }

    public boolean verifButton() {
       return (!mTouchSensor.getState());
    }

    public void calibreActif() {
        pidActive = false;
        m_MotorBras.set(-0.1);
    }

    public void calibreDesactif() {
        init = m_MotorBras.getCurrentPosition();
        setTarget(0);
        pidActive = true;
    }
}



