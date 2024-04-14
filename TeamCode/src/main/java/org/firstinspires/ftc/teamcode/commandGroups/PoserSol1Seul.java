package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.brasCommands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.commands.brasCommands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.pinceCommands.PinceOuvreGaucheCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserSol1Seul extends SequentialCommandGroup {

    public PoserSol1Seul(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasPosCommand bras = new BrasPosCommand(brasSubsystem, 230);
        ExtentionAutoCommand extention = new ExtentionAutoCommand(brasSubsystem, Constants.AutonomousConstants.kExtentionPosSol);
        PinceOuvreGaucheCommand ouvreGauche = new PinceOuvreGaucheCommand(pinceSubsystem);

        addCommands(
                bras,
                extention,
                ouvreGauche
                //poserToile
        );
    }

}