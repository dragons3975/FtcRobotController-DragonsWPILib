package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.OpenPinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PanierAutoCommand extends SequentialCommandGroup {

    public PanierAutoCommand(BrasSubsystem brasSubsystem, PinceBrasSubsystem pinceBrasSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem) {

        ClosePinceBrasCommand ferme = new ClosePinceBrasCommand(pinceBrasSubsystem);
        OpenPinceExtCommand openExt = new OpenPinceExtCommand(pinceExtensionSubsystem);
        BrasPositionCommandtest leverBras = new BrasPositionCommandtest(brasSubsystem, -400, 0.5);
        BrasPositionCommandtest leverBras2 = new BrasPositionCommandtest(brasSubsystem, -550, 1);
        PositionMinPinceBrasCommand pinceMin = new PositionMinPinceBrasCommand(pinceBrasSubsystem);



        addCommands(
                ferme,
                openExt,
                leverBras,
                pinceMin,
                leverBras2
        );
    }

}