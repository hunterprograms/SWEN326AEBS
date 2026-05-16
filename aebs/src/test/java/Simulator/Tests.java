package Simulator;

import swen326.group4.AEBS;

public class Tests{
    
    public static void main(String[] args) {
        try {
            AEBS.start("SC-001", 10_000);
        } catch(InterruptedException ie) {
            ie.printStackTrace();
        }
    }

}
