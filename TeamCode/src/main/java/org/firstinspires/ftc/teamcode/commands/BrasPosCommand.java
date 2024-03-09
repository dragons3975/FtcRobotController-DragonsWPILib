package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class BrasPosCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;
    private int posMoteur = 0;
    double posAvantBras;
    double posRotation;

    public BrasPosCommand(BrasSubsystem brasSubsystem, int _posMoteur, double _posAvantBras, double _posRotation){

        mBrasSubsystem = brasSubsystem;
        addRequirements(brasSubsystem);
        posMoteur = _posMoteur;
        posAvantBras = _posAvantBras;
        posRotation = _posRotation;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       mBrasSubsystem.armGoTo(posMoteur, posRotation, posAvantBras);
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
        return true;
    }
}

