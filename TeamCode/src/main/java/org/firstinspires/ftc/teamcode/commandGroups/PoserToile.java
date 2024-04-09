package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserToile extends SequentialCommandGroup {

    public PoserToile(BrasSubsystem brasSubsystem, PinceSubsystem pinceSubsystem) {

        BrasCommandPos1 bras = new BrasCommandPos1(brasSubsystem, 550);
        ExtentionAutoCommand ext = new ExtentionAutoCommand(brasSubsystem, 37000);
        PinceAutoCommandOuvre ouvre = new PinceAutoCommandOuvre(pinceSubsystem);

        addCommands(
                bras,
                ext,
                ouvre

                //poserToile
        );
    }
}