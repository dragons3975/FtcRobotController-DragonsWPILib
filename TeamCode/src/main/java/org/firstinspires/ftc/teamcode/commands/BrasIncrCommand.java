package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.Command;


public class BrasIncrCommand extends Command{

    private final BrasSubsystem mBrasSubsystem;
    private int incrMoteur = 0;
    double incrAvantBras;
    double incrRotation;

    public BrasIncrCommand(BrasSubsystem brasSubsystem, int _incrMoteur, double _incrAvantBras, double _incrRotation){
        mBrasSubsystem = brasSubsystem;
        addRequirements(brasSubsystem);
        incrMoteur = _incrMoteur;
        incrAvantBras = _incrAvantBras;
        incrRotation = _incrRotation;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       mBrasSubsystem.armPosIncrement(incrMoteur,incrRotation,incrAvantBras);
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

