package org.knowrob.chemlab;

import java.util.HashMap;
import java.util.Vector;

import org.knowrob.interfaces.mongo.types.Designator;
import org.knowrob.prolog.PrologInterface;

import com.google.common.base.CaseFormat;

public class ADTDesignator extends Designator {

	public ADTDesignator() {
		super();
		values.put("type", "ADT");
	}
	
	public Designator readActionChunk(String id) {
		Designator chunkDesignator = new Designator();
		chunkDesignator.put("type", "ADT-action-chunk");
		HashMap<String, Vector<String>> res;

		res = PrologInterface.executeQuery("rdf_has("+id+", rdf:'type', ClassUri), " +
				"rdfs_subclass_of(ClassUri, knowrob:'Event'), " +
				"rdf_split_url(_,ClassName,ClassUri), " +
				"not( ClassName='ActionChunk' )");
		if(res!=null && res.size()>0)
			chunkDesignator.put("action-class", res.get("ClassName").get(0));
		
		res = PrologInterface.executeQuery("rdf_has("+id+", acat:'adtAction', Action)");
		if(res!=null && res.size()>0)
			values.put("adt-action", res.get("Action").get(0));
		else
			System.out.println("ADT " + id + " missing key acat:'adtAction'.");
		
		res = PrologInterface.executeQuery("rdf_has("+id+", knowrob:'taskSuccess', literal(type(_,Success)))");
		if(res!=null && res.size()>0) chunkDesignator.put("success", res.get("Success").get(0));

		res = PrologInterface.executeQuery("event_interval("+id+", T0, T1)");
		if(res!=null && res.size()>0) {
			chunkDesignator.put("start", res.get("T0").get(0));
			chunkDesignator.put("end", res.get("T1").get(0));
		}
		else
			System.out.println("ADT " + id + " missing keys knowrob:'startTime' and knowrob:'endTime'.");
		
		res = PrologInterface.executeQuery("rdf_has("+id+", knowrob:'taskContext', literal(type(_,Context)))");
		if(res!=null && res.size()>0)
			chunkDesignator.put("task", res.get("Context").get(0));
		else
			System.out.println("ADT " + id + " missing key knowrob:'taskContext'.");

		res = PrologInterface.executeQuery("rdf_has("+id+", acat:'adtChunkTrajectory', Traj), " +
				"rdf_has(Traj, knowrob:'relativeTo', RelativeTo)," +
				"rdf_has(Traj, knowrob:'tfFrame', literal(type(_,FrameName)))," +
				"rdf_has(Traj, knowrob:'poseVector', literal(type(_,Vec)))");
		if(res!=null && res.size()>0) {
			chunkDesignator.put("trajectory", res.get("Vec").get(0));
			chunkDesignator.put("trajectory-reference", res.get("RelativeTo").get(0));
			chunkDesignator.put("trajectory-frame", res.get("FrameName").get(0));
		}
		else
			System.out.println("ADT " + id + " underspecified key acat:'adtChunkTrajectory'.");

		res = PrologInterface.executeQuery("rdf_has("+id+", acat:'adtChunkSupport', Support), " +
				"rdf_has(Support, knowrob:'supportedObject', Object)," +
				"rdf_has(Support, knowrob:'translation', literal(type(_,Translation)))," +
				"rdf_has(Support, knowrob:'quaternion', literal(type(_,Quaternion)))");
		if(res!=null && res.size()>0) {
			chunkDesignator.put("supporting-plane-position", res.get("Translation").get(0));
			chunkDesignator.put("supporting-plane-orientation", res.get("Quaternion").get(0));
			chunkDesignator.put("supported-object", res.get("Object").get(0));
		}
		else
			System.out.println("ADT " + id + " underspecified key acat:'acat:adtChunkSupport'.");
		
		return chunkDesignator;
	}

	public Designator readActionChunks(String id) {
		HashMap<String, Vector<String>> res, res2;
		Designator chunkDesignator = new Designator();
		chunkDesignator.put("type", "ADT-action-chunks");
		
		res = PrologInterface.executeQuery("rdf_has('"+id+"', knowrob:'subAction', FirstChunk)");
		if(res!=null) {
			String chunk = res.get("FirstChunk").get(0);
			for(Integer i=0;;++i) {
				chunkDesignator.put(i.toString(), readActionChunk(chunk));
				res2 = PrologInterface.executeQuery(
						"rdf_has("+chunk+", knowrob:'nextAction', NextChunk)");
				if(res2!=null && res2.size()>0) {
					chunk = res2.get("NextChunk").get(0);
				}
				else break;
			}
		}
		return chunkDesignator;
	}

	public Designator readActionRoles(String id) {
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"rdf_has('"+id+"', RoleUri, Object), " +
				"rdfs_subproperty_of(RoleUri, acat:'adtRole')," +
				"rdf_split_url(_,Role,RoleUri)," +
				"adt_object_type(Object, Cls)");
		Designator roleDesignator = new Designator();
		roleDesignator.put("type", "ADT-roles");
		if(res!=null) {
			for(int i=0; res.get("Role").size()>i; ++i) {
				String role = res.get("Role").get(i);
				String cls = res.get("Cls").get(i);
				if(role.startsWith("adt")) role = role.substring(3);
				role = CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, role);
				roleDesignator.put(role, cls);
			}
		}
		return roleDesignator;
	}

	public Designator readObjects(String id) {
		HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
				"rdf_has('"+id+"', acat:'adtObject', Object), " +
				"adt_object_type(Object, Type)");
		Designator objectsDesignator = new Designator();
		objectsDesignator.put("type", "ADT-objects");
		if(res!=null) {
			for(Integer i=0; res.get("Object").size()>i; ++i) {
				HashMap<String, Vector<String>> res2;
				String object = res.get("Object").get(i);
				Designator objectDesignator = new Designator();
				objectDesignator.setType("ADT-object");
				objectDesignator.put("class", res.get("Type").get(i));

				// contains mesh
				res2 = PrologInterface.executeQuery(
						"rdf_has("+object+", knowrob:'pathToCadModel', literal(type(_,Path)))");
				if(res2!=null && res2.size()>0) objectDesignator.put("mesh", res2.get("Path").get(0));
				// contains madeOf
				res2 = PrologInterface.executeQuery(
						"rdf_has("+object+", knowrob:'madeOf', MaterialUri),"+
						"rdf_split_url(_,Material,MaterialUri)");
				if(res2!=null && res2.size()>0) objectDesignator.put("mesh", res2.get("Material").get(0));
				// contains property
				res2 = PrologInterface.executeQuery(
						"rdf_has("+object+", knowrob:'contains', ObjectUri),"+
						"adt_object_type(ObjectUri, Type)");
				if(res2!=null && res2.size()>0) objectDesignator.put("contains", res2.get("Type").get(0));
				
				objectsDesignator.put(i.toString(), objectDesignator);
			}
		}
		return objectsDesignator;
	}

	public Designator readFromIndividual(String id) {
		try {
			values.put("adt-id", id);
			
			HashMap<String, Vector<String>> res = PrologInterface.executeQuery(
					"rdf_has('"+id+"', acat:'adtAction', Action)");
			if(res!=null && res.size()>0)
				values.put("adt-action", res.get("Action").get(0));
			else
				System.out.println("ADT " + id + " missing key acat:'adtAction'.");
			// read instruction
			res = PrologInterface.executeQuery(
					"rdf_has('"+id+"', acat:'instruction', literal(type(_,Instruction)))");
			if(res!=null && res.size()>0)
				values.put("instruction", res.get("Instruction").get(0));
			else
				System.out.println("ADT " + id + " missing key acat:'instruction'.");
			// read actioncore
			res = PrologInterface.executeQuery(
					"rdf_has('"+id+"', acat:'actioncore', literal(type(_,Actioncore)))");
			if(res!=null && res.size()>0)
				values.put("actioncore", res.get("Actioncore").get(0));
			else
				System.out.println("ADT " + id + " missing key acat:'actioncore'.");
			// read time interval
			res = PrologInterface.executeQuery("event_interval('"+id+"', T0, T1)");
			if(res!=null && res.size()>0) {
				values.put("start", res.get("T0").get(0));
				values.put("end", res.get("T1").get(0));
			}
			else
				System.out.println("ADT " + id + " missing keys knowrob:'startTime' and knowrob:'endTime'.");
			// read sub-designators
			values.put("objects", readObjects(id));
			values.put("action-roles", readActionRoles(id));
			values.put("action-chunks", readActionChunks(id));
		}
		catch(Exception e) {
			System.err.println("Parsing ADT designator failed: " + e.getMessage());
			e.printStackTrace();
		}
		
		return this;
	}
}
