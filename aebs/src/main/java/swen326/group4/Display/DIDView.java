package swen326.group4.Display;

import javax.swing.*;
import java.awt.*;


/**
 * DIDView: The graphical representation of the AEBS system.
 * Implements Spec 4.1 (Auditory/Visual alerts) and 4.3 (Success Criteria).
 */
public class DIDView extends JPanel implements AEBSListener {
    private final DIDModel model;
    
    private JLabel speedLabel, distLabel, ttcLabel, statusText, sensitivityLabel;
    private JLabel brakingStatusLabel; // Track ±5% error (Spec 4.3)
    private JLabel maintenanceLight; 
    private JPanel indicator;
    private JSlider sensitivitySlider;

    public DIDView(DIDModel model) {
        this.model = model;
        model.addListener(this);

        setPreferredSize(new Dimension(650, 520));
        setLayout(new BorderLayout(15, 15));
        setBorder(BorderFactory.createEmptyBorder(20, 20, 20, 20));

        setupComponents();
    }

    private void setupComponents() {
        // TOP: Data Panel + Maintenance Light
        JPanel topWrapper = new JPanel(new BorderLayout());
        JPanel stats = new JPanel(new GridLayout(4, 2, 10, 8)); // 4 rows for new data
        
        stats.add(new JLabel("Vehicle Speed (km/h):"));
        speedLabel = new JLabel("0.0");
        stats.add(speedLabel);
        
        stats.add(new JLabel("Object Distance (m):"));
        distLabel = new JLabel("0.0");
        stats.add(distLabel);

        stats.add(new JLabel("Time to Collision (s):"));
        ttcLabel = new JLabel("---");
        stats.add(ttcLabel);

        // Spec 4.3 Success Criteria Display
        stats.add(new JLabel("Braking Error (±5%):"));
        brakingStatusLabel = new JLabel("0.0%");
        stats.add(brakingStatusLabel);

        maintenanceLight = new JLabel(" SERVICE REQ ", SwingConstants.CENTER);
        maintenanceLight.setOpaque(true);
        maintenanceLight.setBackground(Color.LIGHT_GRAY);
        maintenanceLight.setForeground(Color.WHITE);
        maintenanceLight.setFont(new Font("Arial", Font.BOLD, 14));
        maintenanceLight.setBorder(BorderFactory.createLineBorder(Color.BLACK));

        topWrapper.add(stats, BorderLayout.CENTER);
        topWrapper.add(maintenanceLight, BorderLayout.EAST);
        add(topWrapper, BorderLayout.NORTH);

        // CENTER: Visual Indicator (Spec 4.1)
        indicator = new JPanel(new GridBagLayout());
        statusText = new JLabel("SYSTEM READY");
        statusText.setFont(new Font("Monospaced", Font.BOLD, 26));
        indicator.add(statusText);
        add(indicator, BorderLayout.CENTER);

        // SOUTH: Sensitivity Slider (Spec 4.1)
        JPanel southPanel = new JPanel(new BorderLayout());
        sensitivityLabel = new JLabel("Sensitivity Threshold: 1.5s");
        sensitivitySlider = new JSlider(5, 50, 15); 
        sensitivitySlider.addChangeListener(e -> handleSensitivityChange());
        
        southPanel.add(sensitivityLabel, BorderLayout.NORTH);
        southPanel.add(sensitivitySlider, BorderLayout.CENTER);
        add(southPanel, BorderLayout.SOUTH);

        // WEST: Control Buttons
        JButton toggleBtn = new JButton("AEBS ON/OFF");
        toggleBtn.addActionListener(e -> handleStateToggle());
        add(toggleBtn, BorderLayout.WEST);
    }

    private void handleSensitivityChange() {
        if (!sensitivitySlider.getValueIsAdjusting()) {
            double newVal = sensitivitySlider.getValue() / 10.0;
            int response = JOptionPane.showConfirmDialog(this, 
                "Adjust AEBS intervention threshold to " + newVal + "s?",
                "Confirm Sensitivity Change", JOptionPane.YES_NO_OPTION);

            if (response == JOptionPane.YES_OPTION) {
                model.setSensitivityThreshold(newVal);
                sensitivityLabel.setText("Sensitivity Threshold: " + newVal + "s");
            } else {
                sensitivitySlider.setValue((int)(model.getSensitivityThreshold() * 10));
            }
        }
    }

    private void handleStateToggle() {
        DIDModel.SystemState current = model.getSystemState();
        if (current == DIDModel.SystemState.ACTIVE) {
            int response = JOptionPane.showConfirmDialog(this,
                "WARNING: Disabling AEBS will remove emergency braking assistance. Proceed?",
                "Safety Alert", JOptionPane.YES_NO_OPTION, JOptionPane.WARNING_MESSAGE);
            if (response != JOptionPane.YES_OPTION) return;
        }
        DIDModel.SystemState next = (current == DIDModel.SystemState.ACTIVE) ? 
                                     DIDModel.SystemState.DISABLED : DIDModel.SystemState.ACTIVE;
        model.setSystemState(next);
    }

    @Override
    public void stateChanged(DIDModel model) {
        speedLabel.setText(String.format("%.1f", model.getCurrentSpeed()));
        distLabel.setText(String.format("%.1f", model.getDistanceToHazard()));
        ttcLabel.setText(String.format("%.2f", model.getTimeToCollision()));

        // Update Braking Error (Spec 4.3)
        double errorPct = model.getBrakingErrorMargin() * 100;
        brakingStatusLabel.setText(String.format("%.1f%%", errorPct));
        if (Math.abs(errorPct) > 5.0) {
            brakingStatusLabel.setForeground(Color.RED); // Highlight failure to meet spec
        } else {
            brakingStatusLabel.setForeground(new Color(0, 100, 0)); // Dark green for success
        }

        // State Machine for Display Colors
        if (model.getSystemState() == DIDModel.SystemState.MAINTENANCE) {
            maintenanceLight.setBackground(Color.ORANGE);
            maintenanceLight.setForeground(Color.BLACK);
            indicator.setBackground(Color.ORANGE);
            statusText.setText("MAINTENANCE REQ");
        } else {
            maintenanceLight.setBackground(Color.LIGHT_GRAY);
            maintenanceLight.setForeground(Color.WHITE);
            
            if (model.getSystemState() == DIDModel.SystemState.DISABLED) {
                indicator.setBackground(Color.DARK_GRAY);
                statusText.setText("AEBS DEACTIVATED");
            } else {
                updateActiveDisplay(model);
            }
        }
        repaint();
    }

    private void updateActiveDisplay(DIDModel model) {
        // Spec 4.1: Logic for combined visual and auditory notification
        String alarmNote = model.isAlarmActive() ? " + ALARM" : "";

        if (model.isBrakingActive()) {
            indicator.setBackground(Color.RED);
            statusText.setText("BRAKING ENGAGED" + alarmNote);
        } else if (model.getTimeToCollision() <= model.getSensitivityThreshold()) {
            indicator.setBackground(Color.YELLOW);
            statusText.setText("HAZARD WARNING" + alarmNote);
        } else {
            indicator.setBackground(Color.GREEN);
            statusText.setText("SYSTEM ACTIVE");
        }
    }
}