package org.firstinspires.ftc.teamcode.commands.GrimpeurCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class CordePositionCommand extends Command {

    private final GrimpeurCordeSubsystem mGrimpeurCordeSubsystem;

    private double mPosition = 0;


    public CordePositionCommand(GrimpeurCordeSubsystem grimpeurCordeSubsystem, double position) {

        mGrimpeurCordeSubsystem = grimpeurCordeSubsystem;

        mPosition = position;

        addRequirements(grimpeurCordeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mGrimpeurCordeSubsystem.setConsigneCorde(mPosition);
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
        return mGrimpeurCordeSubsystem.isConsigneCorde();
    }
}