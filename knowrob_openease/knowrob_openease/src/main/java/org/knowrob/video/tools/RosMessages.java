package org.knowrob.video;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.jboss.netty.buffer.ChannelBuffer;

/**
 * Message utilities.
 * @author Daniel Beßler
 */
public class RosMessages {
    public static ByteBuffer mapChannelBuffer(final ChannelBuffer channelBuffer) {
        final byte[] data = channelBuffer.array();
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(
        		data.length).order(ByteOrder.nativeOrder());
        byteBuffer.put(data);
        byteBuffer.position(channelBuffer.arrayOffset());
        return byteBuffer;
    }
}
