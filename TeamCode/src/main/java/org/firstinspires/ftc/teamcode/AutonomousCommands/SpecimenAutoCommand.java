package org.firstinspires.ftc.teamcode.AutonomousCommands;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.OpenPinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMaxExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMaxPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class SpecimenAutoCommand extends SequentialCommandGroup {

    public SpecimenAutoCommand(BrasSubsystem brasSubsystem, ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem, PinceBrasSubsystem pinceBrasSubsystem) {


        BrasPositionCommandtest baisserBras = new BrasPositionCommandtest(brasSubsystem, -70, 0.5);
        ExtendPositionCommand extendZero = new ExtendPositionCommand(extensionSubsystem, 10);
        PinceRotationCommand rotationZero = new PinceRotationCommand(pinceExtensionSubsystem, 0);
        PincePositionMaxExtCommand replie = new PincePositionMaxExtCommand(pinceExtensionSubsystem);
        OpenPinceBrasCommand open = new OpenPinceBrasCommand(pinceBrasSubsystem);
        PositionMaxPinceBrasCommand pinceBras = new PositionMaxPinceBrasCommand(pinceBrasSubsystem);

        addCommands(
                extendZero,
                rotationZero,
                replie,
                new WaitCommand(1.5),
                baisserBras,
                pinceBras,
                open
        );
    }

}