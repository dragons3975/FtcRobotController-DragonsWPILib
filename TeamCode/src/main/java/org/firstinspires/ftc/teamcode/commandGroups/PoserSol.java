package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserSol extends SequentialCommandGroup {

    public PoserSol(BrasSubsystem brasSubsystem) {

       BrasCommandPos1 bras = new BrasCommandPos1(brasSubsystem, 230);

        addCommands(
                bras
                //poserToile
        );
    }

}