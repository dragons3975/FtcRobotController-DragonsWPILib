package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.AutonomousCommands.DriveAutonomusCommand;
import org.firstinspires.ftc.teamcode.AutonomousCommands.TurnAutonomousCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class MainAutonomousCommand extends SequentialCommandGroup {

    public MainAutonomousCommand(DriveSubsystem driveSubsystem) {

        DriveAutonomusCommand avance50 = new DriveAutonomusCommand(driveSubsystem, 0,2000);
        DriveAutonomusCommand gauche20 = new DriveAutonomusCommand(driveSubsystem, 500,0);
        //DriveAutonomusCommand recule60 = new DriveAutonomusCommand(driveSubsystem, 0,0);
       // TurnAutonomousCommand tourne90 = new TurnAutonomousCommand(driveSubsystem, 0);

        addCommands(
               // avance50,
                gauche20
               // recule60,
                //tourne90
        );
    }

}
