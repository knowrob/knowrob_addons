package org.knowrob.interfaces.mongo.util;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.GregorianCalendar;

import ros.communication.Time;

public class ISO8601Date {

	protected Date date;

	public ISO8601Date(String isodate) {
		this.date = parse(isodate).getDate();
	}
	
	
	
	public ISO8601Date(Date d) {
		this.date = d;
	}
	
	public ISO8601Date(Time t) {
		date = new Date((long)t.secs * 1000 + t.nsecs/1000);
	}

	
	public static ISO8601Date parse(String datestring) {

		SimpleDateFormat sdf;
		
		if(datestring.contains("."))
			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		else 
			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'");
		
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
	
	public Time getROSTime() {
		GregorianCalendar datum = new GregorianCalendar();
		datum.setTime(date);
		long ms = datum.getTimeInMillis();
		
		Time t  = new Time();
		t.secs  = (int) (ms / 1000);
		t.nsecs = (int) (ms % 1000) * 1000;
		return t;
		
	}
	
}
