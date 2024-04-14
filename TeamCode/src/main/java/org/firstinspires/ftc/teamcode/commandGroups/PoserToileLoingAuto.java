package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasExtentionPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasRotationPosCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PoserToileLoingAuto extends SequentialCommandGroup {

    public PoserToileLoingAuto(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem, DriveSubsystem driveSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne - 20, 0);
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