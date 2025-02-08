package org.firstinspires.ftc.teamcode.commandGroups.Bleu.Droite;

import org.firstinspires.ftc.teamcode.commands.AvanceAutoCommand;
import org.firstinspires.ftc.teamcode.commands.LiftUpCommand;
import org.firstinspires.ftc.teamcode.commands.LiftDownCommand;
import org.firstinspires.ftc.teamcode.commands.PinceCloseCommand;
import org.firstinspires.ftc.teamcode.commands.PinceOpenCommand;
import org.firstinspires.ftc.teamcode.subsystems.BrasSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.PinceSubsystem;


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class BleuDroiteExtra extends SequentialCommandGroup {

    public BleuDroiteExtra(DriveSubsystem driveSubsystem, PinceSubsystem pinceSubsystem, XboxController xboxController, LiftSubsystem liftSubsystem) {
//        pas de pid pour le moment donc avec un timeout
        ParallelRaceGroup avancer = new AvanceAutoCommand(driveSubsystem, -1, 0).withTimeout(0.25);
        ParallelRaceGroup miavancer = new AvanceAutoCommand(driveSubsystem, -1, 0).withTimeout(0.15);
        ParallelRaceGroup mimiavancer = new AvanceAutoCommand(driveSubsystem, -1, 0).withTimeout(0.07);


        ParallelRaceGroup tasserdroite = new AvanceAutoCommand(driveSubsystem, 0, 1).withTimeout(0.25);
        ParallelRaceGroup mitasserdroite = new AvanceAutoCommand(driveSubsystem, 0, 1).withTimeout(0.15);
        ParallelRaceGroup mimitasserdroite = new AvanceAutoCommand(driveSubsystem, 0, 1).withTimeout(0.07);


        ParallelRaceGroup reculer = new AvanceAutoCommand(driveSubsystem, 1, 0).withTimeout(0.3);
        ParallelRaceGroup mireculer = new AvanceAutoCommand(driveSubsystem, 1, 0).withTimeout(0.15);
        ParallelRaceGroup mimireculer = new AvanceAutoCommand(driveSubsystem, 1, 0).withTimeout(0.07);


        ParallelRaceGroup tassergauche = new AvanceAutoCommand(driveSubsystem, 0, -1).withTimeout(0.3);
        ParallelRaceGroup mitassergauche = new AvanceAutoCommand(driveSubsystem, 0, -1).withTimeout(0.15);
        ParallelRaceGroup mimitassergauche = new AvanceAutoCommand(driveSubsystem, 0, -1).withTimeout(0.07);


        PinceOpenCommand pinceopen = new PinceOpenCommand(pinceSubsystem, xboxController);
        PinceCloseCommand pinceclose = new PinceCloseCommand(pinceSubsystem, xboxController);
        LiftUpCommand liftup = new LiftUpCommand(liftSubsystem, xboxController);
//        LiftDownCommand liftdown = new LiftDownCommand(liftSubsystem, xboxController);
//        BrasSubsystem brasup = new BrasSubsystem();


        addCommands(
//                // BLOCK 1
//                avancer,
//                avancer,
//                miavancer,
//                new WaitCommand(1),
//                tasserdroite,
//                new WaitCommand(1),
////                mitasserdroite,
////                new WaitCommand(1),
//                mireculer,
////                mimireculer,
//                new WaitCommand(1),
//                mitassergauche,
//                reculer,
//                reculer,
//                mimireculer,
//
//                new WaitCommand(1),
////
////
////                //BLOCK 2
//                avancer,
//                avancer,
//                miavancer,
//                new WaitCommand(1),
//                tasserdroite,
//                new WaitCommand(1),
//                mireculer,
//                mimireculer,
////                mimireculer,
//                new WaitCommand(1),
//
//                mitassergauche,
//                new WaitCommand(1),
//
//                mireculer,
//                new WaitCommand(1),
//
//                mitassergauche,
//                new WaitCommand(1),
//                reculer,
//                mireculer,
//                new WaitCommand(1),
//
////                //BLOCK 3
////
//                avancer,
//                avancer,
////                avancer,
//                new WaitCommand(1),
//
//                tasserdroite,
//                new WaitCommand(1),
//
//                mireculer,
//                tassergauche,
//                new WaitCommand(1),
//
//                reculer,
//                reculer



//                NEW COOOOOOOOODE
                reculer,
                reculer,
                mireculer,
                new WaitCommand(1),
                mitasserdroite,
                new WaitCommand(1),
                avancer,
                mimiavancer,
                new WaitCommand(1),
                mitassergauche,
                avancer,
                miavancer,
                new WaitCommand(1),

                //SPECIMEN 2

                reculer,
                reculer,
                mireculer,
                new WaitCommand(1),
                mitasserdroite,
                new WaitCommand(1),
                miavancer,
                mimiavancer,
                new WaitCommand(1),
                mitassergauche,
                new WaitCommand(1),
                avancer,
                miavancer,
                mimiavancer

        );
    }

}