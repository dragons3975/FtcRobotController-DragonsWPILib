package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class RamasseCommand extends Command {

    private final IntakeSubsystem mIntakeSubsystem;



    public RamasseCommand(IntakeSubsystem intakeSubsystem) {

        mIntakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mIntakeSubsystem.IntakeDemarrer();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        mIntakeSubsystem.intakeArreter();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
