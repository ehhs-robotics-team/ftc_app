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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.VuforiaScan;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name="DepotAutoWithVuforia", group="Linear Opmode")

public class VuforiaAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor f_leftDrive;
    private DcMotor f_rightDrive;
    private DcMotor b_leftDrive;
    private DcMotor b_rightDrive;   
    private DcMotor motor;
    private Servo leftLift;
    private Servo rightLift;
    private boolean ForwardGo = false;
    private boolean Stop = false;
    //public VuforiaScan vs;
    

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        //vs = new VuforiaScan();

        //vs.initVuforiaTFOD();
        //vs.tfodScan(5);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        f_leftDrive  = hardwareMap.get(DcMotor.class, "f_leftdrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightdrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftdrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightdrive");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        motor = hardwareMap.get(DcMotor.class, "motor");
        
        
        
        

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        f_leftDrive.setDirection(DcMotor.Direction.REVERSE);
        b_leftDrive.setDirection(DcMotor.Direction.REVERSE);
        f_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        b_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(Servo.Direction.REVERSE);
        leftLift.setPosition(0.01);
        rightLift.setPosition(0.01);
        
        boolean forward = false;
        double leftPower = 0.5;
        double rightPower = 0.5;
        
        
        
        
        
        
        

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        leftLift.setPosition(0.01);
        rightLift.setPosition(0.01);
        
        Stop = true;

        RobotDown(3.9);
        Left(0.25);
        Forward(1);
        Right(0.29);
        Forward(0.61);
        PusherDown();
        Backward(0.5);


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //auto starts here
            /*
            if (runtime.seconds() > 0 && runtime.seconds() < 3.1){
                RobotDown();
            }
            if (runtime.seconds() > 3.11 && runtime.seconds() < 3.5){
                Stop();
            }
            if (runtime.seconds() > 3.5 && runtime.seconds() < 3.75){
                Left();
            }
            if (runtime.seconds() > 3.75 && runtime.seconds() < 4.75){
                Forward();
            }
            if (runtime.seconds() > 4.75 && runtime.seconds() < 5.04){
                Right();
            }
            if (runtime.seconds() > 5.04 && runtime.seconds() < 5.65) {
                Forward();
            }
            //puts the pusher down
            if (runtime.seconds() > 5.65 && runtime.seconds() < 6.72){
                Stop();
            }
            if (runtime.seconds() > 6.72 && runtime.seconds() < 7){
                PusherDown();
            }
            if (runtime.seconds() > 7 && runtime.seconds() < 7.5){
                Backward();
            }
            if (runtime.seconds() > 7.5){
                Stop();
            }
            */
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
    
    public void Stop(){
        f_leftDrive.setPower(0);
        f_rightDrive.setPower(0);
        b_leftDrive.setPower(0);
        b_rightDrive.setPower(0);
        motor.setPower(0);
    }

    public void Forward(){
        f_leftDrive.setPower(0.5);
        f_rightDrive.setPower(0.5);
        b_leftDrive.setPower(0.5);
        b_rightDrive.setPower(0.5);
        
    }

    public void Forward(double length){
        double start = runtime.seconds();
        f_leftDrive.setPower(0.5);
        f_rightDrive.setPower(0.5);
        b_leftDrive.setPower(0.5);
        b_rightDrive.setPower(0.5);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }

    public void Backward(){
        f_leftDrive.setPower(-0.5);
        f_rightDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);
    }

    public void Backward(double length){
        double start = runtime.seconds();
        f_leftDrive.setPower(-0.5);
        f_rightDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }

    public void RobotDown() {motor.setPower(-1);}

    public void RobotDown(double length){

        double start = runtime.seconds();
        motor.setPower(-1);

        while (runtime.seconds() < start+length){
            ;
        }
        motor.setPower(0);
    }
    
    public void RobotUp(){
        motor.setPower(1);
    }

    public void RobotUp(double length){

        double start = runtime.seconds();
        motor.setPower(1);
        while (runtime.seconds() < start+length){
            ;
        }
        motor.setPower(0);
    }
    
    public void Left(){
        //left side
        f_leftDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        //right side
        f_rightDrive.setPower(0);
        b_rightDrive.setPower(0);
    }

    public void Left(double length) {
        double start = runtime.seconds();
        //left side
        f_leftDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        //right side
        f_rightDrive.setPower(0);
        b_rightDrive.setPower(0);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }
    
    public void Right(){
        //left side
        f_leftDrive.setPower(0);
        b_leftDrive.setPower(0);
        //right side
        f_rightDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);
    }

    public void Right(double length) {
        double start = runtime.seconds();
        //left side
        f_leftDrive.setPower(0);
        b_leftDrive.setPower(0);
        //right side
        f_rightDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }
    
    public void PusherDown(){
        rightLift.setPosition(-.21);
        leftLift.setPosition(-.2);
   }
   
    public void PusherUp(){
        rightLift.setPosition(0.81);
        leftLift.setPosition(0.8);
   }

  

}
    
    





