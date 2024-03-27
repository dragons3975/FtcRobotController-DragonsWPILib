
package org.firstinspires.ftc.teamcode.subsystems;

import dragons.rev.FtcGyro;
import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class IntakeSubsystem extends Subsystem {

    private final FtcMotor m_Motor = new FtcMotor("ramasseur");



    public IntakeSubsystem() {
        m_Motor.setInverted(true);
    }
    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("ramasseur position", getPosition());
    }


    public void IntakeDemarrer() {
        m_Motor.set(1);
    }

    public void intakeArreter(){
        m_Motor.stopMotor();
    }

    public double getPosition() {
        return m_Motor.getCurrentPosition();
    }

}


