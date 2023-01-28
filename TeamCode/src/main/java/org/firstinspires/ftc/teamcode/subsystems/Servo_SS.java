package org.firstinspires.ftc.teamcode.subsystems;



import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.dragonswpilib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;

public class Servo_SS extends SubsystemBase {

    private Telemetry mTelemetry;
    private HardwareMap mHardwareMap;


    // private final CRServo m_servo;
   // private ServoContinous servo;
    private Servo m_servo;

/*

private final Servo mMoteurPince;

mMoteurPince = mHardwareMap.get(Servo.class, "pince");

mMoteurPince.setPosition(<valeur entre 0 et 1>);





 */


    public Servo_SS(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

       m_servo = hardwareMap.servo.get("Servo");

        //m_servo = mHardwareMap.get(CRServo.class, "Servo");




    }

    @Override
    public void periodic() {

    }


    public void openServo() {

        //m_servo.setPower(-1);
        m_servo.setPosition(1);
    }



    public void closeServo() {
        m_servo.setPosition(0);
    }

    public void closeServo2() {

       // m_servo.setPower(1);
    }


   /* public void servoOFF() {

        m_servo.setPower(0);
    }
*/

}

