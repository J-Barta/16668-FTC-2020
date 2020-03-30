package org.firstinspires.ftc.teamcode.JAVATutorials;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class BuildingBlocks extends LinearOpMode {
    double a;
    boolean b;
    int c;

    public void runOpMode() throws InterruptedException {
        a = 3.14;
        b = true;
        c = 5;

        doesAThing();

        while (c < 100) {
            c += 1;
        }

        a = add1(5);

    }

    public void doesAThing () {
        if (b == false) {
            b = false;
        } else if (a == 6.28) {
            a = 3.15;
        } else {
            c += 1;
        }
    }

    public int add1 (int num) {
        num += 1;
        return num;
    }


}
