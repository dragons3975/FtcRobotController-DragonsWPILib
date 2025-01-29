package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

//import CalibrationCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMaxExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMinExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
//import ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RamasseurCommand extends SequentialCommandGroup {

    public RamasseurCommand(ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem, PinceBrasSubsystem pinceBrasSubsystem, BrasSubsystem brasSubsystem) {


        //command pour que le bras viennent le chercher + extension subsystem rentre
        PincePositionMaxExtCommand replie = new PincePositionMaxExtCommand(pinceExtensionSubsystem);
        ExtendPositionCommand extendZero = new ExtendPositionCommand(extensionSubsystem, 200);
        PinceRotationCommand rotationZero = new PinceRotationCommand(pinceExtensionSubsystem, 1);
        OpenPinceBrasCommand openPinceBras = new OpenPinceBrasCommand(pinceBrasSubsystem);
        ParallelRaceGroup bras = new ParallelRaceGroup(new BrasPositionCommandtest(brasSubsystem, -200, 3), new WaitCommand(2));


        addCommands(
                replie,
                new WaitCommand(0.2),
                extendZero,
                new WaitCommand(0.2),
                rotationZero,
                openPinceBras,
                bras
        );
    }

}