package swen326.group4;

import swen326.group4.Display.*;

import javax.swing.JFrame;

public class Main {
    public static void main(String[] args) {
        JFrame frame = new JFrame("AEBS - Advanced Emergency Braking System - Team 4");

        DIDModel model = new DIDModel();
        DIDView view = new DIDView(model);
       
        frame.add(view);
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.pack();
        frame.setLocationRelativeTo(null);
        frame.setVisible(true);
    }
}