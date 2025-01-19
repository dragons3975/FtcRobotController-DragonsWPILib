package org.firstinspires.ftc.teamcode.commands.GrimpeurCommands;

import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PlieurPositionCommand extends Command {

    private final GrimpeurSubsystem mGrimpeurSubsystem;

    private double mPosition = 0;


    public PlieurPositionCommand(GrimpeurSubsystem grimpeurSubsystem, double position) {

        mGrimpeurSubsystem = grimpeurSubsystem;

        mPosition = position;

        addRequirements(grimpeurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mGrimpeurSubsystem.setConsignePlieur(mPosition);
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
        return mGrimpeurSubsystem.isConsignePlieur();
    }
}