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

/**
 * This file contains an Autonomous OPmode for the EHHS Nerdz 2018-19 robot, Neil Armstrong.
 * It is for either starting position hanging from the lander facing the Depo in the 2018-19
 * Rover Ruckus Competition.
 *
 * This opmode samples both minerals.
 */

@Autonomous(name="Depo Auto Double Mineral", group="Linear Opmode")
//@Disabled
public class AutoDepoDouble extends AutoOP {

    /*
     * Code to run ONCE, called from AutoOP
     */
    @Override
    public void main() {
        initVuforia();
        initTfod();
        //initNavagationTargets();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //encoderLift(LIFT_SPEED, 4,5); //Lift the arm up, lowering the bot.
        //openLock();

        encoderDrive(DRIVE_SPEED,16,16,6);
        encoderTurn(TURN_SPEED, 90);
        encoderDrive(DRIVE_SPEED, -28, -28, 10);

        int GoldPosition = 0;
        for (int x=0; x<2; x++) {
            if (tfodScan(5)) {
                break;
            }
            encoderDrive(DRIVE_SPEED, 16,16, 10);
            GoldPosition = x+1;

        }
        encoderDrive(DRIVE_SPEED, 6,6,3);
        encoderTurn(TURN_SPEED,-90);
        encoderDrive(DRIVE_SPEED, 24, 24, 10);
        encoderTurn(TURN_SPEED, -45);
        encoderDrive(DRIVE_SPEED, 12,12,3);

        ejectMarker();

        encoderDrive(DRIVE_SPEED, -12,-12,3);
        encoderTurn(TURN_SPEED, 45);
        encoderDrive(DRIVE_SPEED, -24, -24, 10);
        encoderTurn(TURN_SPEED,90);

        double landerAvoid = 30;
        double returnDistance1 = landerAvoid+GoldPosition*16;
        double returnDistance2 = landerAvoid+(2-GoldPosition)*16; // Inverse GoldPosition; 2->0 1->1 0->2

        encoderDrive(DRIVE_SPEED, -returnDistance1,-returnDistance1,5);
        encoderTurn(TURN_SPEED, 90);
        encoderDrive(DRIVE_SPEED, returnDistance2, returnDistance2, 5);
        encoderTurn(TURN_SPEED, 90);
        encoderDrive(DRIVE_SPEED, 20, 20, 3);

    }
}