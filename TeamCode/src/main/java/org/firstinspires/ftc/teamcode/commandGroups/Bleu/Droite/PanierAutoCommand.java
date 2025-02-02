package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommand;
import org.firstinspires.ftc.teamcode.commands.BrasCommand.BrasPositionCommandtest;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.CordePositionCommand;
import org.firstinspires.ftc.teamcode.commands.GrimpeurCommands.PlieurPositionCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.ClosePinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.OpenPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMaxPinceBrasCommand;
import org.firstinspires.ftc.teamcode.commands.PinceBrasCommand.PositionMinPinceBrasCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurCordeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GrimpeurSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceBrasSubsystem;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class PanierAutoCommand extends SequentialCommandGroup {

    public PanierAutoCommand(BrasSubsystem brasSubsystem, PinceBrasSubsystem pinceBrasSubsystem) {

        BrasPositionCommandtest leverBras = new BrasPositionCommandtest(brasSubsystem, -450, 0.7);
        PositionMinPinceBrasCommand pinceMin = new PositionMinPinceBrasCommand(pinceBrasSubsystem);



        addCommands(
                leverBras,
                pinceMin
        );
    }

}