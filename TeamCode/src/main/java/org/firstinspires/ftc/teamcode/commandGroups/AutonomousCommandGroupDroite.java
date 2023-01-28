package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.ReculerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerDroiteAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroupDroite extends SequentialCommandGroup {

    public AutonomousCommandGroupDroite(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer90cm = new AvancerAutoCommande(telemetry, driveSubsystem, 73, 0.8);
        TournerDroiteAutoCommande tourner90 = new TournerDroiteAutoCommande(telemetry, driveSubsystem, 90, 0.5);
        AvancerAutoCommande reculer55cm = new AvancerAutoCommande(telemetry, driveSubsystem, 48, 0.5);


            addCommands(
                    avancer90cm,
                    tourner90,
                    reculer55cm
            );

    }

}
