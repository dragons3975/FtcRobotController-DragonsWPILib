//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//
//import org.firstinspires.ftc.dragonswpilib.command.Command;
//import org.firstinspires.ftc.dragonswpilib.command.CommandScheduler;
//
//@Autonomous
//public class RobotAutonomous2 extends OpMode{
//
//    private Command mAutonomousCommand2;
//    private RobotContainer mRobotContainer;
//
//    /*
//     * Code to run ONCE when the driver hits INIT
//     */
//    @Override
//    public void init() {
//        CommandScheduler.getInstance().reset();
//        mRobotContainer = new RobotContainer(gamepad1, gamepad2, telemetry, hardwareMap);
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
//     */
//    @Override
//    public void init_loop() {
//        CommandScheduler.getInstance().run();
//    }
//
//    /*
//     * Code to run ONCE when the driver hits PLAY
//     */
//    @Override
//    public void start() {
//        mAutonomousCommand2 = mRobotContainer.getAutonomousCommand2();
//
//        // schedule the autonomous command (example)
//        if (mAutonomousCommand2 != null) {
//            mAutonomousCommand2.schedule();
//        }
//    }
//
//    /*
//     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
//     */
//    @Override
//    public void loop() {
//        CommandScheduler.getInstance().run();
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//        if (mAutonomousCommand2 != null) {
//            mAutonomousCommand2.cancel();
//        }
//        CommandScheduler.getInstance().reset();
//    }
//}
