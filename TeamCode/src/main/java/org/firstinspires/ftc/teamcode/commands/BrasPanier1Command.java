package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class BrasPanier1Command extends Command{

    private final BrasSubsystem mBrasSubsystem;

    public BrasPanier1Command(BrasSubsystem brasSubsystem) {
        mBrasSubsystem = brasSubsystem;

        addRequirements(brasSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mBrasSubsystem.setConsigne(Constants.BrasConstants.kPositionPanier1);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }


    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // Commande infinie
        return false;
    }
}

