package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.dragonswpilib.command.SequentialCommandGroup;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AvancerAutoCommande;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class AutonomousCommandGroupMilieu extends SequentialCommandGroup {

    public AutonomousCommandGroupMilieu(Telemetry telemetry, DriveSubsystem driveSubsystem) {

        AvancerAutoCommande avancer73cm = new AvancerAutoCommande(telemetry, driveSubsystem, 73, 0.6);


            addCommands(
                    avancer73cm

            );

    }

}
