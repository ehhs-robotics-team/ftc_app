/* Copyright (c) 2018 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
/**
 * This 2018-2019 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the gold and silver minerals.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@TeleOp(name = "Concept: TensorFlow Object Detection", group = "Concept")

public class VuforiaTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
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
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AbEiBY3/////AAAAGbH/Sq+b9ULwoWFe7pLZlcRNDVckrjct+bi1aw5bYvqmY0YPNOfIdPK19cMBDwdyeIMLZ202x5VD0rmxkGWLlVXocop6qzZXp1bbQQMVVKUdXaPOvqnfvbfC9EhJ+Cy9digZVz+F2Cffvm9zZ9RBLIjb3O4i8+b3qBGk3NWQNQYdHLt4f7t9QlsOdU1yyvBTAxvxa7yIzWGlmZHAdbBZpETCiIwaSG7Ykn17FokNPOGHcQ9QvERwUTbp92azytukPOnHRNW2IltM8kd1GFMqMASAii14EIIRvDtqEiQmWhHE0/5qgRmpkK0ZovmgPSRQCg4AOIRUGbWqDTvhIXqAaXtRinO5/Itt9yOZnBLvz0mK";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the Tensor Flow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        f_leftDrive  = hardwareMap.get(DcMotor.class, "f_leftdrive");
        f_rightDrive = hardwareMap.get(DcMotor.class, "f_rightdrive");
        b_leftDrive = hardwareMap.get(DcMotor.class, "b_leftdrive");
        b_rightDrive = hardwareMap.get(DcMotor.class, "b_rightdrive");
        leftLift = hardwareMap.get(Servo.class, "leftLift");
        rightLift = hardwareMap.get(Servo.class, "rightLift");
        motor = hardwareMap.get(DcMotor.class, "motor");
        
        f_leftDrive.setDirection(DcMotor.Direction.REVERSE);
        b_leftDrive.setDirection(DcMotor.Direction.REVERSE);
        f_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        b_rightDrive.setDirection(DcMotor.Direction.FORWARD);
        leftLift.setDirection(Servo.Direction.REVERSE);
        leftLift.setPosition(0.01);
        rightLift.setPosition(0.01);

        waitForStart();
        runtime.reset();
        leftLift.setPosition(0.01);
        rightLift.setPosition(0.01);
        
        Stop = true;
        
        
        
        
        
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                
                
                if (runtime.seconds() > 0 && runtime.seconds() < 3.1){
                RobotDown();
            }
            
            if (runtime.seconds() > 3.11 && runtime.seconds() < 3.5){
                Stop();
            }
            
            if (runtime.seconds() > 3.5 && runtime.seconds() < 3.75){
                Left();
            }
                
                
                
                
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                      telemetry.addData("# Object Detected", updatedRecognitions.size());
                      if (updatedRecognitions.size() == 3) {
                        int goldMineralX = -1;
                        int silverMineral1X = -1;
                        int silverMineral2X = -1;
                        for (Recognition recognition : updatedRecognitions) {
                          if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                          } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                          } else {
                            silverMineral2X = (int) recognition.getLeft();
                          }
                        }
                        if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                          if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Left");
                          } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            telemetry.addData("Gold Mineral Position", "Right");
                          } else {
                            telemetry.addData("Gold Mineral Position", "Center");
                          }
                        }
                      }
                      telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
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
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
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
    
    public void Backward(){
        f_leftDrive.setPower(-0.5);
        f_rightDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);
    }

    public void RobotDown(){
        motor.setPower(-1);
    }
    
    public void RobotUp(){
        motor.setPower(1);
    }
    
    public void Left(){
        
        //left side
        f_leftDrive.setPower(-0.5);
        b_leftDrive.setPower(-0.5);
        //right side
        f_rightDrive.setPower(0);
        b_rightDrive.setPower(0);
    }
    
    public void Right(){
        
        //left side
        f_leftDrive.setPower(0);
        b_leftDrive.setPower(0);
        //right side
        f_rightDrive.setPower(-0.5);
        b_rightDrive.setPower(-0.5);
    }
    
   public void PusherDown(){
       leftLift.setPosition(0.65);
       rightLift.setPosition(0.65);
   }
   
   public void PusherUp(){
       leftLift.setPosition(0.01);
       rightLift.setPosition(0.01);
   }
}
