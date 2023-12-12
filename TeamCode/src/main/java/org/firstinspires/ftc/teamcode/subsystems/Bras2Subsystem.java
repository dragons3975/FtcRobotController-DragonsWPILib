package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.arcrobotics.ftclib.hardware.motors.Motor;

import dragons.rev.FtcMotor;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Bras2Subsystem extends Subsystem {



    //private final FtcMotor m_Motor = new FtcMotor("bras");

    private final FtcMotor m_motor2 = new FtcMotor("bras2");


    //MotorGroup  myMotors;



    // MotorController m_MotorBrasGroup = new MotorController();
   // private final PIDController m_zPID = new PIDController(0.001, 0, 0);

    //private double m_posTarget = 0;

    //private boolean pidActive = false;

    private double init = 0;

    public Bras2Subsystem() {
        //m_zPID.setTolerance(2);


        m_motor2.setInverted(true);

        //myMotors = new MotorGroup(m_motor1, m_motor2);

    }

    @Override
    public void periodic() {
       /* DriverStationJNI.getTelemetry().addData("bouttonApuye", verifButton());
        if (pidActive) {
            double consigne = m_zPID.calculate(m_MotorBras.getCurrentPosition(), m_posTarget);
            DriverStationJNI.getTelemetry().addData("currentPosition", m_MotorBras.getCurrentPosition());
            DriverStationJNI.getTelemetry().addData("target", m_posTarget);
            m_MotorBras.set(consigne);
        }*/
    }

 /*
    public void incrementTarget(double target) {
        //m_posTarget += target;
    }

    public void setTarget(double target) {
        m_posTarget = init + target;
    }

    public boolean verifButton() {
       //return (!mTouchSensor.getState());
    }

    public void calibreActif() {
      //  pidActive = false;
        m_MotorBras.set(-0.1);
    }

   public void calibreDesactif() {
        init = m_MotorBras.getCurrentPosition();
        setTarget(0);
        pidActive = true;
    }*/



/*
    public void up() {
        m_MotorBras.set(0.3);

    }

   public void down() {
        m_MotorBras.set(-0.3);
    }
    public void stop() {
        m_MotorBras.stopMotor();
    }
*/

    public void up() {
        m_motor2.set(0.3);

    }

    public void down() {
        m_motor2.set(-0.3);

    }

    public void stop() {
        m_motor2.stopMotor();
    }





}



