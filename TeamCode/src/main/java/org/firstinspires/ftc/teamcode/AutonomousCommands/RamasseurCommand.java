package org.firstinspires.ftc.teamcode.AutonomousCommands;

//import CalibrationCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.ExtendPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionEchangeExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PincePositionMaxExtCommand;
import org.firstinspires.ftc.teamcode.commands.ExtendCommand.PinceRotationCommand;
//import ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RamasseurCommand extends SequentialCommandGroup {

    public RamasseurCommand(ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem, PinceBrasSubsystem pinceBrasSubsystem, BrasSubsystem brasSubsystem) {


        //command pour que le bras viennent le chercher + extension subsystem rentre
        PincePositionMaxExtCommand replie = new PincePositionMaxExtCommand(pinceExtensionSubsystem);
        ExtendPositionCommand extendZero = new ExtendPositionCommand(extensionSubsystem, 200);
        PinceRotationCommand rotationZero = new PinceRotationCommand(pinceExtensionSubsystem, 0.58);
        OpenPinceBrasCommand openPinceBras = new OpenPinceBrasCommand(pinceBrasSubsystem);
        BrasPositionCommandtest bras1 = new BrasPositionCommandtest(brasSubsystem, -150, 1);
        PositionMinPinceBrasCommand pinceBras = new PositionMinPinceBrasCommand(pinceBrasSubsystem);
        PincePositionEchangeExtCommand positionEchange = new PincePositionEchangeExtCommand(pinceExtensionSubsystem);

        addCommands(
                replie,
                new WaitCommand(0.2),
                extendZero,
                new WaitCommand(0.2),
                rotationZero,
                openPinceBras,
                pinceBras,
                bras1,
                positionEchange
        );
    }

}