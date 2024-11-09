package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserToileProcheAuto extends SequentialCommandGroup {

    public PoserToileProcheAuto(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileProche - 20, 0);
        LeveBrasHauteurToileCommand leve = new LeveBrasHauteurToileCommand(brasSubsystem, pinceSubsystem);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, 20, 0);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);

        addCommands(
                avancer,
                leve,
                avancer2,
                new WaitCommand(0.5)
        );
    }
}