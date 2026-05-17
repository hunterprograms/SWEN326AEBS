package Simulator;

import java.io.IOException;

import Simulator.ScenarioGenerator.ScenarioConfig;
import Simulator.ScenarioGenerator.ScenarioGenerator;
import Simulator.ScenarioGenerator.ScenarioLibrary;
import swen326.group4.AEBS;

public class Tests{
    
    public static void main(String[] args) {
        try {
            new ScenarioGenerator(ScenarioLibrary.highwayRearEnd(), "/home/hunter/projects/SWEN326AEBS/scenarios/SC-002").generate();
        } catch(IOException ioe){}

        try {
            // AEBS.start("SC-001", 10_000);
            AEBS.start("SC-002", 10_000);
            // AEBS.start("SC-003", 8_000);
            // AEBS.start("SC-004", 10_000);
            // AEBS.start("SC-005", 12_000);
            // AEBS.start("SC-006", 12_000);
            // AEBS.start("SC-007", 8_000);
            // AEBS.start("SC-008", 8_000);
            // AEBS.start("SC-009", 10_000);
            // AEBS.start("SC-010", 6_000);
        } catch(InterruptedException ie) {
            ie.printStackTrace();
        }
    }

}
