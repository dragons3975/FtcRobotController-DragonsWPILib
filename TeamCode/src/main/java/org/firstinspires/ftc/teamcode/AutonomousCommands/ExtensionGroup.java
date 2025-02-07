package org.firstinspires.ftc.teamcode.AutonomousCommands;

//import CalibrationCommand;

import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.OpenPinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMinExtCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ExtensionGroup extends SequentialCommandGroup {

    public ExtensionGroup(ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem) {
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        //CalibrationCommand calib = new CalibrationCommand(extensionSubsystem);
        //ExtendCommand extensionXcm = new ExtendCommand(extensionSubsystem);
        //OpenPinceExtCommand ramasse = new OpenPinceExtCommand(pinceExtensionSubsystem);
        ExtendPositionCommand extendOut = new ExtendPositionCommand(extensionSubsystem, 0);
        PincePositionMinExtCommand plie = new PincePositionMinExtCommand(pinceExtensionSubsystem);
        OpenPinceExtCommand pinceOpen = new OpenPinceExtCommand(pinceExtensionSubsystem);
        ClosePinceExtCommand pinceClose = new ClosePinceExtCommand(pinceExtensionSubsystem);



        addCommands(
                //reculer,
                //new WaitCommand(0.2),
                //calib,
                //extensionXcm,
                extendOut,
                plie,
                pinceOpen,
                pinceClose

        );
    }

}