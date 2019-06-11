package org.openease.client;

/**
 * Holds errors which occur during communication with openEASE.
 * 
 * @author Moritz Horstmann <mhorst@cs.uni-bremen.de>
 *
 */
public class EASEError extends Exception {
    /**
     * openEASE error with error message
     * 
     * @param msg
     *            error message
     */
    public EASEError(String msg) {
        super(msg);
    }

    /**
     * openEASE error with error message and cause
     * 
     * @param msg
     *            error message
     */
    public EASEError(String msg, Throwable e) {
        super(msg, e);
    }

    private static final long serialVersionUID = 1191750418076452736L;

}
