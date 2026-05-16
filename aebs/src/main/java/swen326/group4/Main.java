package swen326.group4;

import swen326.group4.Audio.AuditoryController;
import swen326.group4.Display.*;

import javax.swing.JFrame;
import javax.swing.SwingUtilities;

public class Main {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            // 1. Setup the core components
            DIDModel model = new DIDModel();
            DIDView view = new DIDView(model);
            DIDController controller = new DIDController(model);

            // 2. Connect the audio listener
            AuditoryController audio = new AuditoryController();
            model.addListener(audio);

            // 3. Setup the UI Window
            JFrame frame = new JFrame("AEBS Monitoring System - NORMAL");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.add(view);
            frame.pack();
            frame.setVisible(true);

            /* * NORMAL STATE:
             * Braking: false
             * Alarm:   false
             * Distance: 80.0 meters (Safe)
             */
            controller.updateInterventionMetrics(false, false, 0.0);
            
            System.out.println("AEBS initialized: System is in NORMAL monitoring mode.");
        });
    }
}