package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.ReculerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerAutoCommande;
import org.firstinspires.ftc.teamcode.commands.TournerDroiteAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Autonomous2CommandGroupGauche extends SequentialCommandGroup {

    public Autonomous2CommandGroupGauche(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer73cm = new AvancerAutoCommande(telemetry, driveSubsystem, 73, 0.5);
        TournerDroiteAutoCommande tourner90 = new TournerDroiteAutoCommande(telemetry, driveSubsystem, 90, 0.5);
        ReculerAutoCommande avancer55cm = new ReculerAutoCommande(telemetry, driveSubsystem, 48, 0.5);


            addCommands(
                    avancer73cm,
                    tourner90,
                    avancer55cm
            );

    }

}
