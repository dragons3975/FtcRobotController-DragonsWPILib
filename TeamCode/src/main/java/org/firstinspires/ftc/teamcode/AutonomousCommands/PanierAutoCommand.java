package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PanierAutoCommand extends SequentialCommandGroup {

    public PanierAutoCommand(BrasSubsystem brasSubsystem, PinceBrasSubsystem pinceBrasSubsystem) {

        BrasPositionCommandtest leverBras = new BrasPositionCommandtest(brasSubsystem, -450, 0.5);
        PositionMinPinceBrasCommand pinceMin = new PositionMinPinceBrasCommand(pinceBrasSubsystem);



        addCommands(
                leverBras,
                pinceMin
        );
    }

}