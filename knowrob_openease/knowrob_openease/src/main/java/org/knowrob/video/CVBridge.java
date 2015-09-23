package org.knowrob.video;

import static com.googlecode.javacv.cpp.opencv_core.CV_16UC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_16UC3;
import static com.googlecode.javacv.cpp.opencv_core.CV_16UC4;
import static com.googlecode.javacv.cpp.opencv_core.CV_32FC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_32FC2;
import static com.googlecode.javacv.cpp.opencv_core.CV_32FC3;
import static com.googlecode.javacv.cpp.opencv_core.CV_32FC4;
import static com.googlecode.javacv.cpp.opencv_core.CV_64FC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_64FC2;
import static com.googlecode.javacv.cpp.opencv_core.CV_64FC3;
import static com.googlecode.javacv.cpp.opencv_core.CV_64FC4;
import static com.googlecode.javacv.cpp.opencv_core.CV_8UC1;
import static com.googlecode.javacv.cpp.opencv_core.CV_8UC2;
import static com.googlecode.javacv.cpp.opencv_core.CV_8UC3;
import static com.googlecode.javacv.cpp.opencv_core.CV_8UC4;
import static com.googlecode.javacv.cpp.opencv_core.cvMat;

import org.knowrob.video.ImageEncodings;

import com.googlecode.javacpp.Pointer;
import com.googlecode.javacv.cpp.opencv_core.CvMat;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import org.jboss.netty.buffer.ChannelBuffer;

/**
 * Mapping of ROS Image messages to OpenCV images.
 * Based on ros_bridge cpp code.
 * @author Daniel Beßler
 */
public class CVBridge {
	public static enum Format {
		INVALID(-1),
		GRAY(0),
		RGB(1),
		BGR(2),
		RGBA(3),
		BGRA(4),
		YUV422(5),
		BAYER_RGGB(6),
		BAYER_BGGR(7),
		BAYER_GBRG(8),
		BAYER_GRBG(9);
		
	    int val;
	    Format(int val)
	    { this.val = val; }
	};
	
	public static int getCvType(final String encoding) throws Exception {
	  // Color formats
	  if (encoding.equals(ImageEncodings.BGR8))   return CV_8UC3;
	  if (encoding.equals(ImageEncodings.RGB8))   return CV_8UC3;
	  if (encoding.equals(ImageEncodings.BGR16))  return CV_16UC3;
	  if (encoding.equals(ImageEncodings.RGB16))  return CV_16UC3;
	  if (encoding.equals(ImageEncodings.BGRA8))  return CV_8UC4;
	  if (encoding.equals(ImageEncodings.RGBA8))  return CV_8UC4;
	  if (encoding.equals(ImageEncodings.BGRA16)) return CV_16UC4;
	  if (encoding.equals(ImageEncodings.RGBA16)) return CV_16UC4;
	  //
	  if (encoding.equals(ImageEncodings.MONO8))  return CV_8UC1;
	  if (encoding.equals(ImageEncodings.MONO16)) return CV_16UC1;
	  // Floating point formats
	  if (encoding.equals(ImageEncodings.TYPE_32FC1)) return CV_32FC1;
	  if (encoding.equals(ImageEncodings.TYPE_32FC2)) return CV_32FC2;
	  if (encoding.equals(ImageEncodings.TYPE_32FC3)) return CV_32FC3;
	  if (encoding.equals(ImageEncodings.TYPE_32FC4)) return CV_32FC4;
	  if (encoding.equals(ImageEncodings.TYPE_64FC1)) return CV_64FC1;
	  if (encoding.equals(ImageEncodings.TYPE_64FC2)) return CV_64FC2;
	  if (encoding.equals(ImageEncodings.TYPE_64FC3)) return CV_64FC3;
	  if (encoding.equals(ImageEncodings.TYPE_64FC4)) return CV_64FC4;
	  // For bayer, return one-channel
	  if (encoding.equals(ImageEncodings.BAYER_RGGB8))  return CV_8UC1;
	  if (encoding.equals(ImageEncodings.BAYER_BGGR8))  return CV_8UC1;
	  if (encoding.equals(ImageEncodings.BAYER_GBRG8))  return CV_8UC1;
	  if (encoding.equals(ImageEncodings.BAYER_GRBG8))  return CV_8UC1;
	  if (encoding.equals(ImageEncodings.BAYER_RGGB16)) return CV_16UC1;
	  if (encoding.equals(ImageEncodings.BAYER_BGGR16)) return CV_16UC1;
	  if (encoding.equals(ImageEncodings.BAYER_GBRG16)) return CV_16UC1;
	  if (encoding.equals(ImageEncodings.BAYER_GRBG16)) return CV_16UC1;
	  // Miscellaneous
	  if (encoding.equals(ImageEncodings.YUV422)) return CV_8UC2;

	  throw new Exception("Unrecognized image encoding [" + encoding + "]");
	}

	public static Format getFormat(final String encoding) {
	  if ((encoding.equals(ImageEncodings.MONO8)) ||
			  (encoding.equals(ImageEncodings.MONO16)))  return Format.GRAY;
	  if ((encoding.equals(ImageEncodings.BGR8))  ||
			  (encoding.equals(ImageEncodings.BGR16)))   return Format.BGR;
	  if ((encoding.equals(ImageEncodings.RGB8))  ||
			  (encoding.equals(ImageEncodings.RGB16)))   return Format.RGB;
	  if ((encoding.equals(ImageEncodings.BGRA8)) ||
			  (encoding.equals(ImageEncodings.BGRA16)))  return Format.BGRA;
	  if ((encoding.equals(ImageEncodings.RGBA8)) ||
			  (encoding.equals(ImageEncodings.RGBA16)))  return Format.RGBA;
	  if (encoding.equals(ImageEncodings.YUV422)) return Format.YUV422;

	  if ((encoding.equals(ImageEncodings.BAYER_RGGB8)) ||
			  (encoding.equals(ImageEncodings.BAYER_RGGB16))) return Format.BAYER_RGGB;
	  if ((encoding.equals(ImageEncodings.BAYER_BGGR8)) ||
			  (encoding.equals(ImageEncodings.BAYER_BGGR16))) return Format.BAYER_BGGR;
	  if ((encoding.equals(ImageEncodings.BAYER_GBRG8)) ||
			  (encoding.equals(ImageEncodings.BAYER_GBRG16))) return Format.BAYER_GBRG;
	  if ((encoding.equals(ImageEncodings.BAYER_GRBG8)) ||
			  (encoding.equals(ImageEncodings.BAYER_GRBG16))) return Format.BAYER_GRBG;

	  // We don't support conversions to/from other types
	  return Format.INVALID;
	}
	
	/**
	 * Map data from Image message into OpenCV matrix.
	 * @param rosImgMsg The Image message.
	 * @return CvMat with mapped data.
	 * @throws Exception
	 */
	public static CvMat getMappedCvMat(sensor_msgs.Image rosImgMsg) throws Exception {
	  int source_type = getCvType(rosImgMsg.getEncoding());
	  int byte_depth = ImageEncodings.bitDepth(rosImgMsg.getEncoding()) / 8;
	  int num_channels = ImageEncodings.numChannels(rosImgMsg.getEncoding());

	  if (rosImgMsg.getStep() < rosImgMsg.getWidth() * byte_depth * num_channels) {
		  StringBuilder sb = new StringBuilder();
		  sb.append("Image is wrongly formed: step < width * byte_depth * num_channels  or  ");
		  sb.append(rosImgMsg.getStep());
		  sb.append(" != ");
		  sb.append(rosImgMsg.getWidth());
		  sb.append(" * ");
		  sb.append(byte_depth);
		  sb.append(" * ");
		  sb.append(num_channels);
		  throw new Exception(sb.toString());
	  }
	  
	  final Pointer matData = new Pointer(
			  CVBridge.mapChannelBuffer(rosImgMsg.getData()));
	  return cvMat(
			  rosImgMsg.getHeight(),
			  rosImgMsg.getWidth(),
			  source_type,
			  matData);
	}
	
    public static ByteBuffer mapChannelBuffer(final ChannelBuffer channelBuffer) {
        final byte[] data = channelBuffer.array();
        ByteBuffer byteBuffer = ByteBuffer.allocateDirect(
        		data.length).order(ByteOrder.nativeOrder());
        byteBuffer.put(data);
        byteBuffer.position(channelBuffer.arrayOffset());
        return byteBuffer;
    }
}
