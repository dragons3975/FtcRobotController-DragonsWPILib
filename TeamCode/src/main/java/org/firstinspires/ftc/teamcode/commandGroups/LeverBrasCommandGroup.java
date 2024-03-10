package org.firstinspires.ftc.teamcode.commandGroups;

import org.firstinspires.ftc.teamcode.commands.BrasPosCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class LeverBrasCommandGroup extends SequentialCommandGroup {



    public LeverBrasCommandGroup(BrasSubsystem brasSubsystem) {

        BrasPosCommand command1 = new BrasPosCommand(brasSubsystem, -1,-1,0.525);
        WaitCommand wait = new WaitCommand(0.3);
        BrasPosCommand command2 = new BrasPosCommand(brasSubsystem, 350,300,-1);
        WaitCommand wait2 = new WaitCommand(0.8);
        BrasPosCommand command3 = new BrasPosCommand(brasSubsystem, 775,690,0.245);
        /*
        BrasPosCommand command1 = new BrasPosCommand(brasSubsystem, -1,-1,0.525);
        WaitCommand wait = new WaitCommand(0.5);

        BrasPosCommand command2 = new BrasPosCommand(brasSubsystem,-1,230,-1);
        WaitCommand wait2 = new WaitCommand(0.05);
        BrasPosCommand command3 = new BrasPosCommand(brasSubsystem,217,-1,-1);
        WaitCommand wait3 = new WaitCommand(0.05);

        BrasPosCommand command4 = new BrasPosCommand(brasSubsystem,-1,460,-1);
        WaitCommand wait4 = new WaitCommand(0.05);
        BrasPosCommand command5 = new BrasPosCommand(brasSubsystem,434,-1,-1);
        WaitCommand wait5 = new WaitCommand(0.05);

        BrasPosCommand command6 = new BrasPosCommand(brasSubsystem,-1,690,0.245);
        WaitCommand wait6 = new WaitCommand(0.05);
        BrasPosCommand command7 = new BrasPosCommand(brasSubsystem,650,-1,-1);*/



        addCommands(

                command1,
                wait,
                command2,
                wait2,
                command3/*,

                wait3,
                command4,
                wait4,
                command5,
                wait5,
                command6,
                wait6,
                command7*/
        );
    }
}