package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;

public class TogglePositionDepartCommand extends CommandBase {

    private final ConfigSubsystem mConfigSubsystem;

    public TogglePositionDepartCommand(ConfigSubsystem configSubsystem){
        mConfigSubsystem = configSubsystem;
        addRequirements(configSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        mConfigSubsystem.togglePositionDepart();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
