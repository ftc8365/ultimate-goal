package org.firstinspires.ftc.teamcode.ultimategoal.robot;

public class Constraint {

    double targetPower  = 0.30;
    boolean rampUp      = true;
    boolean rampDown    = true;
    boolean stopMotor   = false;

    public Constraint() {
    }

    public double getTargetPower() {
        return this.targetPower;
    }

    public Constraint setTargetPower( double power ) {
        this.targetPower = power;
        return this;
    }

    public boolean getRampUp() {
        return this.rampUp;
    }

    public Constraint setRampUp( boolean rampUp ) {
        this.rampUp = rampUp;
        return this;
    }

    public boolean getRampDown() {
        return this.rampDown;
    }

    public Constraint setRampDown( boolean rampDown ) {
        this.rampDown = rampDown;
        return this;
    }

    public boolean getStopMotor() {
        return this.stopMotor;
    }

    public Constraint setStopMotor( boolean stopMotor ) {
        this.stopMotor = stopMotor;
        return this;
    }
}
