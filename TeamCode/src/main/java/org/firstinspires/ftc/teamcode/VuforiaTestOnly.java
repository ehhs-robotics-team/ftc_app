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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

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

public class VuforiaTestOnly extends LinearOpMode {

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

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AbEiBY3/////AAAAGbH/Sq+b9ULwoWFe7pLZlcRNDVckrjct+bi1aw5bYvqmY0YPNOfIdPK19cMBDwdyeIMLZ202x5VD0rmxkGWLlVXocop6qzZXp1bbQQMVVKUdXaPOvqnfvbfC9EhJ+Cy9digZVz+F2Cffvm9zZ9RBLIjb3O4i8+b3qBGk3NWQNQYdHLt4f7t9QlsOdU1yyvBTAxvxa7yIzWGlmZHAdbBZpETCiIwaSG7Ykn17FokNPOGHcQ9QvERwUTbp92azytukPOnHRNW2IltM8kd1GFMqMASAii14EIIRvDtqEiQmWhHE0/5qgRmpkK0ZovmgPSRQCg4AOIRUGbWqDTvhIXqAaXtRinO5/Itt9yOZnBLvz0mK";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    public VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    public TFObjectDetector tfod;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
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
        rightLift.setDirection(Servo.Direction.REVERSE);

        f_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        f_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        b_rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        boolean forward = false;
        double leftPower = 0.5;
        double rightPower = 0.5;

        PusherUp();
        initVuforiaTFOD();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        while (opModeIsActive()) {
            if (tfodScan(30)){
                ;
            }

        }

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive() && runtime.seconds() <= 30) {
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

    public void Stop(double length) {
        double start = runtime.seconds();
        Stop();
        while (runtime.seconds() < start+length){
            ;
        }
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
        f_rightDrive.setPower(0.5);
        b_rightDrive.setPower(0.5);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }

    public void Left(double length, boolean frontOnly) {
        double start = runtime.seconds();
        //left side
        f_leftDrive.setPower(-0.5);
        //right side
        f_rightDrive.setPower(0.5);

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
        f_leftDrive.setPower(0.5);
        b_leftDrive.setPower(0.5);
        //right side
        f_rightDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);

        while (runtime.seconds() < start+length){
            ;
        }
        Stop();
    }
    
    public void PusherDown(){
        rightLift.setPosition(0.15);
        leftLift.setPosition(0.14);
   }
   
    public void PusherUp(){
        rightLift.setPosition(0.81);
        leftLift.setPosition(0.8);
   }

   public void HoldMarker(){
        rightLift.setPosition(0.41);
        leftLift.setPosition(0.4);
   }


    // Vuforia and TFOD methods
    public void initVuforiaTFOD() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    public void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        telemetry.addData("Vuforia works", true);
        telemetry.update();
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    public void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Scan for one minerals and returns true if its the gold mineral.
     * true:  Gold
     * false: Silver
     */
    public boolean tfodScan(int timeoutS) {

        boolean returnValue = false;
        runtime.reset();

        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }

        while (opModeIsActive() && runtime.seconds() < timeoutS) {
            telemetry.addData("TFOD", "Running scan");
            telemetry.update();
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                //updatedRecognitions.clear();
                //updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());
                    if (updatedRecognitions.size() >= 1) {
                        //for (Recognition recognition : updatedRecognitions) {
                        Recognition recognition = updatedRecognitions.get(0);//updatedRecognitions.size()-1);
                        String label = recognition.getLabel();
                        telemetry.addData("mineral is:",label);
                        telemetry.update();
                        sleep(1000);
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            returnValue = true;
                            return returnValue;
                        } else if (recognition.getLabel().equals(LABEL_SILVER_MINERAL)) {
                            returnValue = false;
                            return returnValue;
                        } else {
                            returnValue = false;
                            return returnValue;
                        }
                        //}
                    }
                    telemetry.update();
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
        return returnValue;
    }

}
    
    





