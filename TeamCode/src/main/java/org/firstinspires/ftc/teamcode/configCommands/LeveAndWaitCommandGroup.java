package org.firstinspires.ftc.teamcode.configCommands;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ConfigSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeveAndWaitCommandGroup extends SequentialCommandGroup {

    public LeveAndWaitCommandGroup(BrasSubsystem brasSubsystem, ConfigSubsystem configSubsystem) {

        addCommands(
                new BrasRotationPosCommand(brasSubsystem, Constants.BrasConstants.kHauteurCamera).withTimeout(10),
                new WaitCommand(2),
                new SetConfigReadyCommand(configSubsystem)
        );
    }

}
