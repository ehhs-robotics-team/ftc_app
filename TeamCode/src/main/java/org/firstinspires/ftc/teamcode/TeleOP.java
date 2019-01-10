/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="TeleOP Parent", group="Iterative Opmode")
@Disabled
public abstract class TeleOP extends OpMode {
    // Declare OpMode members.
    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    public DcMotor armDrive = null;
    public DcMotor lift_arm = null;

    public Servo liftLock = null;
    public CRServo intakeServo = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        armDrive = hardwareMap.get(DcMotor.class, "bench_max");
        lift_arm = hardwareMap.get(DcMotor.class, "lift_arm");
        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftLock = hardwareMap.get(Servo.class, "lift_lock");
        intakeServo = hardwareMap.get(CRServo.class, "intake_servo"); // Must be in continous rotation mode

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);
        lift_arm.setDirection(DcMotor.Direction.FORWARD);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        main();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



    /**
     * method for children autonomous opmodes to override and insert case specific moves.
     */
    public abstract void main();
    // e.g.
    //        drive();
    //        armMotion();
    //        intake();
    //        liftMotion();
    //        setLiftLockContinuous();
    //        checkEmergencyStop();


    /*
     * Reads and sets variables for the drive controls
     */
    public void drive() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower;
        double rightPower;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        double drive = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        leftPower = Range.clip(drive + turn, -1.0, 1.0);
        rightPower = Range.clip(drive - turn, -1.0, 1.0);


        // Tank Mode uses one stick to control each wheel.
        // - This requires no math, but it is hard to drive forward slowly and keep straight.
        // leftPower  = -gamepad1.left_stick_y ;
        // rightPower = -gamepad1.right_stick_y ;

        // Square the power to reduce sensitivity in the center.
        leftPower = leftPower*Math.abs(leftPower);
        rightPower = rightPower*Math.abs(rightPower);

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Drive Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
    public void servoIn() {
        if (gamepad1.left_bumper) {
            intakeServo.setPower(-0.7);
            telemetry.addData("Intake:" , "In");
        }
        else if (gamepad1.right_bumper) {
            intakeServo.setPower(0);
            telemetry.addData("Intake", "Out");
        }
        else if (gamepad1.y) {
            intakeServo.setPower(0.7);
            telemetry.addData("Intake", "Stop");
        }
        telemetry.update();
    }

    public void armMotion() {
        double liftSpeed = 1.0;
        // Setup a variable for the arm drive to save power level for telemetry
        double armPower;
        // left is forward, right is backward, and they cancel each other out.
        armPower =  gamepad1.left_trigger-gamepad1.right_trigger;
        armDrive.setPower(armPower);

        //Also loosen/tighten the lift arm to take up the slack created.
        if (armPower > 0) {
            lift_arm.setPower(-liftSpeed);
        }
        else if(armPower<0){
            lift_arm.setPower(liftSpeed);
        }
        else if(armPower < 0.5 && armPower > -0.5){ // Don't change the lift position with less than half power.
            lift_arm.setPower(0);
        }
        telemetry.addData("Arm Power:", armPower);
    }

    public void intake() {
        // Setup a variable for the intake drive to save power level for telemetry
        double forwardSpeed = 1.0;
        double backwardSpeed = 0.0;
        double stopSpeed = 0.5;
        // Right is forward (in), Left is backward (out).
        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            intakeServo.setPower(stopSpeed);
            telemetry.addData("Intake:", "OFF");
        }
        else if (gamepad1.right_bumper) {
            intakeServo.setPower(forwardSpeed);
            telemetry.addData("Intake:", "FORWARD");
        }
        else if (gamepad1.left_bumper) {
            intakeServo.setPower(backwardSpeed);
            telemetry.addData("Intake:", "BACKWARD");
        }
    }

    public void liftMotion() {
        // Declare a variable for the lift speed
        double liftSpeed =0.5;
        //Up moves the arm up, down moves it down;
        if (gamepad1.dpad_up) {
            lift_arm.setPower(liftSpeed);
        }
        else if (gamepad1.dpad_down) {
            lift_arm.setPower(-liftSpeed);
        }
        else {
            lift_arm.setPower(0);
        }

    }

    /**
     * Method for operating the sevo lock at the top of the lifting arm.
     * Assumes a regular, 180 degree servo.
     */
    public void setLiftLockPosition() {
        // Right should close the lock, Left to open it.
        if (gamepad1.dpad_right) {
            liftLock.setPosition(1.0);
            telemetry.addData("lock", "shut");
        }
        else if (gamepad1.dpad_left) {
            liftLock.setPosition(0);
            telemetry.addData("lock", "open");
        }
        telemetry.update();
    }

    /**
     * Method for operating the sevo lock at the top of the lifting arm.
     * Assumes a continuous rotation servo.
     */
    public void setLiftLockContinuous() {
        // Right should close the lock, Left to open it.
        if (gamepad1.dpad_right) {
            liftLock.setPosition(.75);
        }
        else if (gamepad1.dpad_left) {
            liftLock.setPosition(.25);
        }
    }

    /**
     * Method for stopping all motors in the case of emergency
     */
    public void checkEmergencyStop() {
        if (gamepad1.x) {
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            armDrive.setPower(0);
            lift_arm.setPower(0);
            intakeServo.setPower(0.5);
            //liftLock.setPosition(0.0); // only use this if the servo is acting as a continuous rotation servo
            telemetry.addData("Emergency Stop", "Driver pressed 'x'");
            telemetry.update();
        }

    }

}
