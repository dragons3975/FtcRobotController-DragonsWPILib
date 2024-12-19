package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class BrasSubsystem extends Subsystem {

    private final FtcMotor mMotorBras = new FtcMotor("bras");

    //private final ArduinoServo mServoBras = new ArduinoServo(Constants.MotorPortConstants.kBrasServo);

    private double mSpeed = 0;

    //private double mAngle = 0;

    PIDController mPid = new PIDController(0.01, 0, 0);

    private int mConsigne = 0;

    public BrasSubsystem() {
        stop();
        mMotorBras.setInverted(true);
    }

    @Override
    public void periodic() {

        //mArduinoMotorBras.set(mSpeed);
        //mServoBras.setAngle(mAngle);
        //DriverStationJNI.Telemetry.putNumber("servo bras position", mServoBras.getAngle());
        DriverStationJNI.getTelemetry().addData("Encodeur Bras base", mMotorBras.getCurrentPosition());
        //DriverStationJNI.Telemetry.putNumber("Encodeur Bras servo", mServoBras.getTachoCount());

        double output = mPid.calculate(mMotorBras.getCurrentPosition(), mConsigne);

        //if(output >= 0.7) {
        //    output = 0.7;
        //}
        //if(output <= 0.0) {
         //   output = 0.0;
        //}

        DriverStationJNI.getTelemetry().addData("IsConsigne t/f bras", isConsigne());
        DriverStationJNI.getTelemetry().addData("Output bras", output);
        DriverStationJNI.getTelemetry().addData("consigne", mConsigne);
        mMotorBras.set(output);

    }

    public void setSpeed(double speed) {
        mSpeed = speed;
    }

    public double getPos() {
        return mMotorBras.getCurrentPosition();
    }

    public void incrementTargetRotation(double inc) {
        mConsigne += inc;
    }

    public void stop() {
        mConsigne = mMotorBras.getCurrentPosition();
    }

    public void setConsigne(int consigne) {
        mConsigne = consigne;
    }

    public void incrementConsigne(double incr) {
        mConsigne += incr;
    }

    public boolean isConsigne() {
        mPid.setTolerance(10);
        return mPid.atSetpoint();
    }

}