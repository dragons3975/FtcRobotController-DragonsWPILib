package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Command;

public class GrimperCommand extends Command {

    private final GrimpeurSubsystem mGrimpeurSubsystem;


    public GrimperCommand(GrimpeurSubsystem grimpeurSubsystem) {

        mGrimpeurSubsystem = grimpeurSubsystem;

        addRequirements(grimpeurSubsystem);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        mGrimpeurSubsystem.incrementConsigne(50);
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
