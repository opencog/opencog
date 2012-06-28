/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javadestin;

/**
 *
 * @author ted
 */
public interface IExperiment {
    public void stop();
    
    public void start();
    
    public void setPOSTraining(boolean training);
    
    public void setPSSATraining(boolean training);
    
    public void reset();
}
