package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.ReculerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Autonomous2CommandGroupDroite extends SequentialCommandGroup {

    public Autonomous2CommandGroupDroite(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer90cm = new AvancerAutoCommande(telemetry, driveSubsystem, 73, 0.8);
        TournerAutoCommande tourner90 = new TournerAutoCommande(telemetry, driveSubsystem, 90, 0.5);
        ReculerAutoCommande reculer55cm = new ReculerAutoCommande(telemetry, driveSubsystem, 48, 0.5);


            addCommands(
                    avancer90cm,
                    tourner90,
                    reculer55cm
            );

    }

}
