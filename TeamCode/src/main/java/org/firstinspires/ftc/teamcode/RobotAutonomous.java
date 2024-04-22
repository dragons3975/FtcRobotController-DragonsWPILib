package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Autonomous(preselectTeleOp = "Robot")
public class RobotAutonomous extends OpMode{

    private Command mAutonomousCommand;
    private RobotContainer mRobotContainer;
    private boolean mAutoScheduled = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        DriverStationJNI.init(gamepad1, gamepad2, telemetry, hardwareMap, DriverStationJNI.Modes.auto);
        mRobotContainer = new RobotContainer();
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        DriverStation.refreshData();
        CommandScheduler.getInstance().run();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        mRobotContainer.init();
        mRobotContainer.getWaitCommand().schedule();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (mAutoScheduled == false) {
            mAutonomousCommand = mRobotContainer.getAutonomousCommand();

            if (mAutonomousCommand != null) {
                mAutonomousCommand.schedule();
                mAutoScheduled = true;
            }
        }

        DriverStation.refreshData();
        CommandScheduler.getInstance().run();
        /*try {
            Thread.sleep(19); //Deja un sleep de 1ms dans le internalRunOpMode. On veut du 20ms a peu près... Voir pour utiliser un timer.
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }*/

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        if (mAutonomousCommand != null) {
            mAutonomousCommand.cancel();
        }
        CommandScheduler.getInstance().reset();
    }
}
