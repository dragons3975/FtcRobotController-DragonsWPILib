package org.firstinspires.ftc.teamcode.commands.pinceCommands;


import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class PinceToggleInclinaisonCommand extends Command{

    private final PinceSubsystem mPinceSubsystem;

    public PinceToggleInclinaisonCommand(PinceSubsystem pinceSubsystem) {
        mPinceSubsystem = pinceSubsystem;

        addRequirements(pinceSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mPinceSubsystem.InclineToggle();
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

