package swen326.group4.Car;

import javax.swing.*;
import java.awt.Color;

/**
 * 
 */
public class Interface
{
    public static void main(String[] args){

        
        JPanel panel = new JPanel();
        panel.setBackground(Color.gray);
        panel.setBounds(0, 0, 500, 250);

        JFrame frame = new JFrame();
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(null);
        frame.setSize(1000, 500);
        frame.setVisible(true);
        frame.add(panel);

    }





}
