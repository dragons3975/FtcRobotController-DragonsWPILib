package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommandPos1;
import org.firstinspires.ftc.teamcode.commands.ExtentionAutoCommand;
import org.firstinspires.ftc.teamcode.commands.PinceAutoCommandOuvre;
import org.firstinspires.ftc.teamcode.commands.RamasseAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TranslaterAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class PoserSol extends SequentialCommandGroup {

    public PoserSol(IntakeSubsystem intakeSubsystem, BrasSubsystem brasSubsystem) {

       BrasCommandPos1 bras = new BrasCommandPos1(brasSubsystem, 230);
       RamasseAutoCommand ramasse = new RamasseAutoCommand(intakeSubsystem, 5000);

        addCommands(
                bras,
                ramasse
                //poserToile
        );
    }

}