package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class CustomPID {

    private DcMotor motor;
    private double Kp;
    private double Ki;
    private double Kd;
    private int target;
    private boolean running = true;
    ElapsedTime timer = new ElapsedTime();
    double integralSum = 0.0;
    double lastError = 0;
    int oldTarget = target;
    boolean override = false;
    int position;
    PIDFController controller;

    public CustomPID(DcMotor motor, double kp, double ki, double kd, int target) {
        this.motor = motor;
        Kp = kp;
        Ki = ki;
        Kd = kd;
        this.target = target;

        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDFController(new PIDCoefficients(Kp, Ki, Kd));

    }

    public void update() {

        position = -(int)(motor.getCurrentPosition());
        double error = target - position;


//        motor.setPower(controller.update);

//        if (oldTarget != target) {
//            running = true;
//        } else if (Math.abs(error) < 10) {
//            running = false;
//        }
//        if (!running && Math.abs(error) > 50) {
//            running = true;
//        }

        if (running) {

            double derivative = (error - lastError) / timer.seconds();
            integralSum = integralSum + (error * timer.seconds());
            double out = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

            if (!override)
                motor.setPower(out);


            lastError = error;

            timer.reset();

        } else {
            motor.setPower(0);
        }

        oldTarget = target;
    }

    public int getTarget() {
        return target;
    }

    public void setTarget(int target) {
        this.target = target;
        override = false;
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean isRunning() {
        return running;
    }

    public void setKp(double kp) {
        Kp = kp;
    }

    public void setKi(double ki) {
        Ki = ki;
    }

    public void setKd(double kd) {
        Kd = kd;
    }

    public void setRunning(boolean running) {
        this.running = running;
    }

    public int getPosition() {
        return position;
    }

    public void setPower(double power) {
        override = true;
        motor.setPower(power);
    }
    public boolean isOverride() {
        return override;
    }
}