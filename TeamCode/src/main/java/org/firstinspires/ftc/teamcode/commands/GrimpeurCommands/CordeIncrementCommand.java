package org.firstinspires.ftc.teamcode.commands.GrimpeurCommands;

import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class CordeIncrementCommand extends Command {

    private final GrimpeurCordeSubsystem mGrimpeurCordeSubsystem;

    private double mIncrment;


    public CordeIncrementCommand(GrimpeurCordeSubsystem grimpeurCordeSubsystem, double increment) {

        mGrimpeurCordeSubsystem = grimpeurCordeSubsystem;

        mIncrment = increment;

        addRequirements(grimpeurCordeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mGrimpeurCordeSubsystem.incrementConsigneCorde(mIncrment);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}