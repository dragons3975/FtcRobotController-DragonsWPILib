package org.firstinspires.ftc.teamcode.commandGroups.Rouge.Gauche;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commandGroups.PoserToile;
import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.TourneAutoCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class RougeGaucheTeamPropGauche extends SequentialCommandGroup {

    public RougeGaucheTeamPropGauche(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        AvanceAutoCommand avancer = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementInitial, 0);
        AvanceAutoCommand avancer2 = new AvanceAutoCommand(driveSubsystem, Constants.AutonomousConstants.kAvancementVersToileEloigne, 0);
        TourneAutoCommand tourne = new TourneAutoCommand(driveSubsystem, 90);
        PoserToile poserToile = new PoserToile(brasSubsystem, pinceSubsystem);



        addCommands(
                avancer,
                tourne,
                avancer2,
                poserToile
        );
    }

}