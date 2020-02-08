/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.*;
import java.net.*;

/**
 * Add your docs here.
 */
public class PowerCellSocket {

    public static double x, y;

    public static void callSocket()
    {
        try {
            Socket s = new Socket("169.254.107.120", 5805);
            DataInputStream dis = new DataInputStream(s.getInputStream());
            String str = (String) dis.readUTF();
            x = Double.parseDouble(str.split(",")[0].replaceAll("\\s", ""));
            y = Double.parseDouble(str.split(",")[1].replaceAll("\\s", ""));
            s.close();
        }
        catch (Exception e){
            System.out.println(e);
        }
    }

    public static double getX(){
        return x;
    }

    public static double getY(){
        return y;
    }
}
