package org.firstinspires.ftc.teamcode.commands.GrimpeurCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;

public class PlieurDefaultCommand extends Command {

    private final GrimpeurSubsystem mGrimpeurSubsystem;

    private final GrimpeurCordeSubsystem mGrimpeurCordeSubsystem;
    private final XboxController mXboxController2;


    public PlieurDefaultCommand(GrimpeurSubsystem grimpeurSubsystem, GrimpeurCordeSubsystem grimpeurCordeSubsystem, XboxController xboxController2) {

        mGrimpeurSubsystem = grimpeurSubsystem;

        mGrimpeurCordeSubsystem = grimpeurCordeSubsystem;

        mXboxController2 = xboxController2;

        addRequirements(grimpeurSubsystem);
        addRequirements(grimpeurCordeSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (mXboxController2.getRightBumper()) {
            mGrimpeurSubsystem.incrementConsignePlieur(Constants.GrimpeurConstants.GrimperValue);
        }
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