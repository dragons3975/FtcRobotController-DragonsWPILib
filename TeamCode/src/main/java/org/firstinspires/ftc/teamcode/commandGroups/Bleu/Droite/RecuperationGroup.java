package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

//import CalibrationCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
//import ClosePinceExtCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceExtensionSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RecuperationGroup extends SequentialCommandGroup {

    public RecuperationGroup(ExtensionSubsystem extensionSubsystem, PinceExtensionSubsystem pinceExtensionSubsystem, PinceBrasSubsystem pinceBrasSubsystem) {
        //pas de pid pour le moment donc avec un tiBleuDroiteExtrameout
        //ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, -1, 0, 0).withTimeout(0.3);
        //ClosePinceExtCommand rentrer = new ClosePinceExtCommand(pinceExtensionSubsystem);

        //command pour que le bras viennent le chercher + extension subsystem rentre
        OpenPinceBrasCommand prendre = new OpenPinceBrasCommand(pinceBrasSubsystem);
        ClosePinceBrasCommand fermer = new ClosePinceBrasCommand(pinceBrasSubsystem);


        addCommands(
                //reculer,
                //new WaitCommand(0.2),
                //rentrer,
                prendre,
                fermer
        );
    }

}