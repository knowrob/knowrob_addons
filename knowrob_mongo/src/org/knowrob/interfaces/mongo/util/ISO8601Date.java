package org.knowrob.interfaces.mongo.util;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.GregorianCalendar;

public class ISO8601Date {

	protected Date date;
	
	
	public ISO8601Date(Date d) {
		this.date = d;
	}
	
	
	public static ISO8601Date parse(String datestring) {
		
	    SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
	    GregorianCalendar datum = new GregorianCalendar();
	    try {
	    	datum.setTime(sdf.parse(datestring));
	    	return new ISO8601Date(datum.getTime());
	    } catch (ParseException e) {
		    e.printStackTrace();
	    }
	    return null;
	}

	
	@Override
	public String toString() {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		return sdf.format(this.date);
	}

	
	public long toNanoSeconds() {
		return date.getTime() * 1000;
	}
	
	public Date getDate() {
		return date;
	}


	public void setDate(Date date) {
		this.date = date;
	}
}
