package org.firstinspires.ftc.teamcode.subsystems;


import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import dragons.rev.FtcTouchSensor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class BrasSubsystem extends Subsystem {


    private final FtcMotor m_MotorBras = new FtcMotor("motor0");
    private final FtcServo m_ServoRotationPince = new FtcServo("servo1");
    private final FtcServo m_ServoAvantBras = new FtcServo("servo2");

    private double m_PosRotationPince = 0;
    private double m_PosAvantBras = 0.4;
    private int m_PosBrasMoteur;

    public BrasSubsystem() {
        m_PosBrasMoteur = m_MotorBras.getCurrentPosition();
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Encodeur bras", m_MotorBras.getCurrentPosition());
        DriverStationJNI.getTelemetry().addData("Variable Moteur", m_PosBrasMoteur);
        DriverStationJNI.getTelemetry().addData("Variable PosAvantBras", m_PosAvantBras);
        DriverStationJNI.getTelemetry().addData("Variable PosRotationPince", m_PosRotationPince);
        m_MotorBras.setTargetPosition(m_PosBrasMoteur);
        m_ServoRotationPince.setPosition(m_PosRotationPince);
        m_ServoAvantBras.setPosition(m_PosAvantBras);
    }

    public void armPosition(int position){
        if (position == 0) { //TODO mettre les positions aux moteurs

        }
    }
    public void armPosIncrement(int motorBras, double servoRotationPince, double servoAvantBras){
        m_PosBrasMoteur += motorBras;
        m_PosAvantBras += servoAvantBras;
        m_PosRotationPince += servoRotationPince;
    }
    public void armGoTo(int motorBras, double servoRotationPince, double servoAvantBras){
        if (motorBras != -1){
            m_PosBrasMoteur = motorBras;
        }
        if (servoAvantBras != -1){
            m_PosAvantBras = servoAvantBras;
        }
        if (servoRotationPince != -1){
            m_PosRotationPince = servoRotationPince;
        }
    }

}



