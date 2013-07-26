package org.knowrob.interfaces.mongo.util;

import java.text.ParseException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.GregorianCalendar;

import ros.communication.Time;

public class ISO8601Date {

	protected Date date;

	
	// constructors
	public ISO8601Date(String isodate) {
		SimpleDateFormat sdf;
		
		if(isodate.contains("."))
			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		else 
			sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss'Z'");
		
	    GregorianCalendar datum = new GregorianCalendar();
	    try {
	    	datum.setTime(sdf.parse(isodate));
	    	this.date = datum.getTime();
	    	
	    } catch (ParseException e) {
		    e.printStackTrace();
	    }

	}
	
	public ISO8601Date(Date d) {
		this.date = d;
	}

	public ISO8601Date(Time t) {
		date = new Date((long)t.secs * 1000 + t.nsecs/1000);
	}

	public ISO8601Date(long t) {
		date = new Date(t);
	}

	
	
	@Override
	public String toString() {
		SimpleDateFormat sdf = new SimpleDateFormat("yyyy-MM-dd'T'HH:mm:ss.SSS'Z'");
		return sdf.format(this.date);
	}

	public Date getDate() {
		return date;
	}

	public long getNanoSeconds() {
		return date.getTime() * 1000;
	}

	public long getMilliSeconds() {
		return date.getTime() * 1000;
	}
	
	public Time toROSTime() {
		GregorianCalendar datum = new GregorianCalendar();
		datum.setTime(date);
		long ms = datum.getTimeInMillis();
		
		Time t  = new Time();
		t.secs  = (int) (ms / 1000);
		t.nsecs = (int) (ms % 1000) * 1000;
		return t;	
	}
	
}
