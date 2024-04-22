package org.firstinspires.ftc.teamcode.configCommands;

import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class SetConfigReadyCommand extends Command {

    private final ConfigSubsystem mConfigSubsystem;

    public SetConfigReadyCommand(ConfigSubsystem configSubsystem) {
        mConfigSubsystem = configSubsystem;

        addRequirements(configSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mConfigSubsystem.setReady();
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
