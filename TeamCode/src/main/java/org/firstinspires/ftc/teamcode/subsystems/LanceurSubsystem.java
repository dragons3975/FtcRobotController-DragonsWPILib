package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.Constants;

import dragons.rev.FtcMotor;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class LanceurSubsystem extends Subsystem {

    private final FtcMotor mMotorTest4 = new FtcMotor("lanceur");
    private double mSpeed = 0;
    private int mEncPrec;
    private boolean mPeriodicTestBool = false;
    private int mPeriodicTestNb;

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
    }

    public void monte() {
        mMotorTest4.set(1);
    }
    public void setSpeed(double speed) {
        mSpeed = speed;
        mMotorTest4.set(mSpeed);
    }

    public void stop() {
        mMotorTest4.stopMotor();
    }
    public void TestPerStart() {
        mPeriodicTestBool = true;
    }
    public void TestPerEnd() {
        mPeriodicTestBool = false;
    }

}