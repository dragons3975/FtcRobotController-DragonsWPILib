package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

//import CalibrationCommand;

import org.firstinspires.ftc.teamcode.commands.ExtendCommand.OpenPinceExtCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ExtensionGroup extends SequentialCommandGroup {

    public ExtensionGroup(ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        //CalibrationCommand calib = new CalibrationCommand(extensionSubsystem);
        //ExtendCommand extensionXcm = new ExtendCommand(extensionSubsystem);
        OpenPinceExtCommand ramasse = new OpenPinceExtCommand(pinceExtensionSubsystem);


        addCommands(
                //reculer,
                //new WaitCommand(0.2),
                //calib,
                //extensionXcm,
                ramasse

        );
    }

}