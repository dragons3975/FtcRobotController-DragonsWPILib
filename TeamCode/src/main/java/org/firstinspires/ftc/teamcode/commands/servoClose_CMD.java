package org.firstinspires.ftc.teamcode.commands;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.dragonswpilib.command.CommandBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Servo_SS;

public class servoClose_CMD extends CommandBase {

    private final Servo_SS mServo_SS;
    private final Telemetry mTelemetry;

    private final Gamepad mGamepad;
    private  double sevoDegrees;

    public servoClose_CMD(Telemetry telemetry, Servo_SS p_Servo_SS, Gamepad gamepad/*, double p_ServoSpeed*/){
        mTelemetry = telemetry;
        mServo_SS  = p_Servo_SS;
        mGamepad = gamepad;
        //sevoDegrees = p_ServoSpeed;

        addRequirements(p_Servo_SS);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute( ) {

       // double upSpeed= liftSpeed;
        mServo_SS.closeServo();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

      //  mServo_SS.servoOFF();

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
