package org.firstinspires.ftc.teamcode.commands.GrimpeurCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class GrimpeurDefaultCommand extends Command {

    private final GrimpeurSubsystem mGrimpeurSubsystem;
    private final XboxController mXboxController2;


    public GrimpeurDefaultCommand(GrimpeurSubsystem grimpeurSubsystem, XboxController xboxController2) {

        mGrimpeurSubsystem = grimpeurSubsystem;

        mXboxController2 = xboxController2;

        addRequirements(grimpeurSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mGrimpeurSubsystem.incrementConsigne(Constants.GrimpeurConstants.GrimperValue);
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