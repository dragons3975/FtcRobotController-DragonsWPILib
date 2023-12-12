package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;


import dragons.rev.FtcServo;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PlaneLauncher_SS extends Subsystem {


private final FtcServo m_servoMotor = new FtcServo("servo_avion");



    private double init = 0;

    public PlaneLauncher_SS() {
        //m_zPID.setTolerance(2);

     //   servoMotor = hardwareMap.get(Servo.class,"servo_avion");
        planeLaucherOFF();

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

    public void planeLaucherOn(){
        m_servoMotor.setPosition(0.5);

    }

    public void planeLaucherOFF(){
        m_servoMotor.setPosition(0.0);

    }







}



