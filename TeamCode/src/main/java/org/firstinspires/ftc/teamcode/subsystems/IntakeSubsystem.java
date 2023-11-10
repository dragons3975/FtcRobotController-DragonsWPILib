
package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem extends Subsystem {

    private final FtcMotor m_MotorDroit = new FtcMotor("Intakeright");
    private final FtcMotor m_MotorGauche = new FtcMotor("Intakeleft");



    public IntakeSubsystem() {
    m_MotorDroit.setInverted(true);
    }


    public void IntakeDemarrer() {
        m_MotorDroit.set(1);
        m_MotorGauche.set(1);
    }

    public void intakeArreter(){
        m_MotorDroit.stopMotor();
        m_MotorGauche.stopMotor();
    }

    @Override
    public void periodic() {

    }

    public void stop () {
        m_MotorDroit.stopMotor();
        m_MotorGauche.stopMotor();
    }
}



