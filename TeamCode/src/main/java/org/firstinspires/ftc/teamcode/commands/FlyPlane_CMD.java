package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PlaneLauncher_SS;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;


public class FlyPlane_CMD extends Command{

    private final PlaneLauncher_SS m_PlaneLauncher_SS;

    private final XboxController mxBoxController;

    private int mXSpeed;
    private int mZRotation;

    private double targetAvant = 0;

    public FlyPlane_CMD(PlaneLauncher_SS p_PlaneLauncher_SS, XboxController xboxController) {
        m_PlaneLauncher_SS = p_PlaneLauncher_SS;
        mxBoxController = xboxController;

        addRequirements(p_PlaneLauncher_SS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_PlaneLauncher_SS.planeLaucherOFF();


    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_PlaneLauncher_SS.planeLaucherOn();

    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_PlaneLauncher_SS.planeLaucherOFF();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie car la commande sera appellée avec un withTimeout()
        // donc elle sera interrompue à la fin du timeout
        return false;
    }
}

