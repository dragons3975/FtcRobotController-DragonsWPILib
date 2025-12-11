package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;

import edu.wpi.first.math.controller.PIDController;


public class LanceurSubsystem extends Subsystem {

    private final FtcMotor mMotorTest4 = new FtcMotor("lanceur");
    private double mSpeed = 0;
    private int mEncPrec;
    private boolean mPeriodicTestBool = false;
    private int mPeriodicTestNb;
    private final PIDController pid = new PIDController(Constants.LanceurConstants.kP, 0, 0);

    private double mSetPointVit;



    public LanceurSubsystem() {
        mMotorTest4.setInverted(true);
    }

    @Override
    public void periodic() {
        DriverStationJNI.getTelemetry().addData("Motor lanceur speed", mMotorTest4.get());
        DriverStationJNI.getTelemetry().addData("Motor lanceur encodeur", mMotorTest4.getCurrentPosition());

        int encAct = mMotorTest4.getCurrentPosition();

        int deltaEnc = encAct - mEncPrec;

        double vitesseEnTickParSec = deltaEnc/Constants.LanceurConstants.kPeriodeSec;

        double vitesseEnTourParSec = vitesseEnTickParSec/(Constants.LanceurConstants.kTicksParTourRev);

        mEncPrec = encAct;

        DriverStationJNI.getTelemetry().addData("TickParSec", vitesseEnTickParSec);
        DriverStationJNI.getTelemetry().addData("TourParSec", vitesseEnTourParSec);

        if (mPeriodicTestBool) {mPeriodicTestNb += 1;}

        DriverStationJNI.getTelemetry().addData("PeriodicBool", mPeriodicTestBool);
        DriverStationJNI.getTelemetry().addData("PeriodicPar10Sec", mPeriodicTestNb);
        DriverStationJNI.getTelemetry().addData("PeriodicParSec", mPeriodicTestNb/10);

        double output = pid.calculate(vitesseEnTourParSec, mSetPointVit);

        mMotorTest4.set(mMotorTest4.get() + output);


        DriverStationJNI.getTelemetry().addData("pidOutput", pid.calculate(vitesseEnTourParSec, mSetPointVit));
        DriverStationJNI.getTelemetry().addData("vitAct", mMotorTest4.get());
        DriverStationJNI.getTelemetry().addData("vitAct", mSetPointVit);

        //motor.set(vitesse actuel + output)
    }

    public void monte() {
        mMotorTest4.set(1);
    }
    public void setSpeed(double speed) {
        mSpeed = speed;
        mMotorTest4.set(mSpeed);
    }

    public void setTPS(double speed) {
        mSetPointVit = speed;
        mMotorTest4.set(mSetPointVit);
    }

    public void stop() {
        mSetPointVit = 0;
        mMotorTest4.stopMotor();
    }
    public void TestPerStart() {
        mPeriodicTestBool = true;
    }
    public void TestPerEnd() {
        mPeriodicTestBool = false;
    }

    public void pidTest() {
        mSetPointVit = 1;
    }

}