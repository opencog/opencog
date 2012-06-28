/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package javadestin;

import callbacks.BeliefGraphCallback;

/**
 *
 * @author teds
 */
public class NetworkFactory implements INetworkFactory {

    private boolean useOriginal = true;

    public NetworkFactory(boolean useOriginal) {
        this.useOriginal = useOriginal;
    }

    @Override
    public INetwork create() {
        if (useOriginal) {

            NetworkConfig config = new NetworkConfig();
            INetwork n = new Network(new int[]{20, 16, 14, 12, 10, 8, 6, 2}, 16, config); //4,6,8,10,12,14,16,20
            n.setIterationFinishedCallback(new BeliefGraphCallback());
            return n;
        } else {
            return new NetworkAlt(SupportedImageWidths.W512, 8, new long[]{20, 16, 14, 12, 10, 8, 6, 2});
        }
    }
}
