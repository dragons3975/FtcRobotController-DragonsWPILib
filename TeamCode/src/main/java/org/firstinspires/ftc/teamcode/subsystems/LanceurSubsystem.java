
package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;
import dragons.rev.FtcServo;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LanceurSubsystem extends Subsystem {

    private final FtcServo m_Servo = new FtcServo("lanceur");

    private boolean lance = false;



    public LanceurSubsystem() {
        m_Servo.setPosition(0.2);
    }

    public void lance() {
        if (!lance) {
            m_Servo.setPosition(1);
            lance = true;
        } else {
            m_Servo.setPosition(0.2);
            lance = false;
        }
    }



    @Override
    public void periodic() {

    }


}



