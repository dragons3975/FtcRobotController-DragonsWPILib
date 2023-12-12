package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Arm_SS;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class Arm_Up_CMD extends Command{

    private final Arm_SS m_Arm_SS;

    private final XboxController mxBoxController;

    private int mXSpeed;
    private int mZRotation;

    private double targetAvant = 0;

    private double speedX = 0.8;
    private double speedY = 0;

    public Arm_Up_CMD(Arm_SS p_Arm_SS, XboxController xboxController) {
        m_Arm_SS = p_Arm_SS;
        mxBoxController = xboxController;

        addRequirements(p_Arm_SS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        DriverStationJNI.getTelemetry().addData("Init", "ASDASDAS");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_Arm_SS.up(speedX,speedY);

        DriverStationJNI.getTelemetry().addData("joystick", mxBoxController.getRightY());
       // mBrasSubsystem.incrementTarget(mxBoxController.getRightY() * 2);
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_Arm_SS.stop();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie car la commande sera appellée avec un withTimeout()
        // donc elle sera interrompue à la fin du timeout
        return false;
    }
}

