package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.commands.DriveCommand.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.CordePositionCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurPositionCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class GrimpeAutoCommand extends SequentialCommandGroup {

    public GrimpeAutoCommand(GrimpeurSubsystem grimpeurSubsystem, GrimpeurCordeSubsystem grimpeurCordeSubsystem) {

        PlieurPositionCommand deplier = new PlieurPositionCommand(grimpeurSubsystem, -4000);
        CordePositionCommand grimper = new CordePositionCommand(grimpeurCordeSubsystem, -20000);
        PlieurPositionCommand plier = new PlieurPositionCommand(grimpeurSubsystem, 0);


        addCommands(
                grimper.alongWith(new WaitCommand(1), plier)
        );
    }

}