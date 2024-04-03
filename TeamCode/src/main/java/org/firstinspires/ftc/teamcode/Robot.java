package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@TeleOp
public class Robot extends OpMode {

    private RobotContainer mRobotContainer;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        DriverStationJNI.init(gamepad1, gamepad2, telemetry, hardwareMap, DriverStationJNI.Modes.teleop);
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
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
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
        mRobotContainer.stop();
        CommandScheduler.getInstance().reset();
    }
}
