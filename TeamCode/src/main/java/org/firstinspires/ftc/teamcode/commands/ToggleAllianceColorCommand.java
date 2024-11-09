package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

public class ToggleAllianceColorCommand extends Command{
    ConfigSubsystem mConfigSubsystem;
    public ToggleAllianceColorCommand(ConfigSubsystem configSubsystem) {
        mConfigSubsystem = configSubsystem;
        addRequirements(configSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mConfigSubsystem.toggleAllianceColor();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
