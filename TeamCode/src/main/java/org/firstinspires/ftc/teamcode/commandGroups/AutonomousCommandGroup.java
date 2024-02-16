package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.DriveAutoCommand;
import org.firstinspires.ftc.teamcode.commands.FermePinceCommand;
import org.firstinspires.ftc.teamcode.commands.GoToPositionBrasCommand;
import org.firstinspires.ftc.teamcode.commands.OuvrePinceCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonomousCommandGroup extends SequentialCommandGroup {

    public AutonomousCommandGroup(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, BrasSubsystem brasSubsystem) {

        ParallelRaceGroup avancer4sec = new DriveAutoCommand(driveSubsystem, 0, 1, 0).withTimeout(4);
        ParallelRaceGroup avancer2sec = new DriveAutoCommand(driveSubsystem, 0, 1, 0).withTimeout(2.5);

        ParallelRaceGroup tourner1sec = new DriveAutoCommand(driveSubsystem, 0, 0, 1).withTimeout(1);
        ParallelRaceGroup reculer4sec = new DriveAutoCommand(driveSubsystem, 0, -1, 0).withTimeout(4);
        FermePinceCommand ferme = new FermePinceCommand(pinceSubsystem);
        OuvrePinceCommand ouvre = new OuvrePinceCommand(pinceSubsystem);
        WaitCommand attendre1sec = new WaitCommand(1);
        WaitCommand attendre2sec = new WaitCommand(1);
        ParallelRaceGroup allerDroite5sec = new DriveAutoCommand(driveSubsystem, 1, 0, 0).withTimeout(5);
        ParallelRaceGroup allerDroite3sec = new DriveAutoCommand(driveSubsystem, 1, 0, 0).withTimeout(3);

        GoToPositionBrasCommand goTo200 = new GoToPositionBrasCommand(brasSubsystem, 200);

        addCommands(
                ferme,
                attendre1sec,
                avancer4sec,
                attendre2sec,
                allerDroite5sec,
                tourner1sec,
                goTo200,
                allerDroite3sec,
                avancer2sec,
                ouvre,
                reculer4sec
        );
    }


}
