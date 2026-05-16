package Simulator;

import swen326.group4.AEBS;

public class Tests{
    
    public static void main(String[] args) {
        try {
            // AEBS.start("SC-001", 10_000);
            // AEBS.start("SC-002", 6_000);
            // AEBS.start("SC-003", 8_000);
            // AEBS.start("SC-004", 10_000);
            // AEBS.start("SC-005", 12_000);
            // AEBS.start("SC-006", 12_000);
            // AEBS.start("SC-007", 8_000);
            // AEBS.start("SC-008", 8_000);
            AEBS.start("SC-009", 10_000);
            //AEBS.start("SC-010", 6_000);
        } catch(InterruptedException ie) {
            ie.printStackTrace();
        }
    }

}
