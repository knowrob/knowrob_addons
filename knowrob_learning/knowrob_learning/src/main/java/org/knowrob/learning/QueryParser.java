package org.knowrob.learning;
public class QueryParser
{
	public static String parseAPrologQueryFromDesignator(String queryDesignator) 
  	{
		int index = queryDesignator.indexOf(":QUERY") + 6;

		String subquery = queryDesignator.substring(index, queryDesignator.length() - 1);
		subquery = subquery.replaceAll("\\\\\",\\\\\",", "");
		subquery = subquery.replaceAll("term", "");
		subquery = subquery.replaceAll("\\{", "");
		subquery = subquery.replaceAll("\\}", "");
		subquery = subquery.replaceAll("\\\\\"\\\\\":", "");
		subquery = subquery.replaceAll("\\[\\[", "\\[");
		subquery = subquery.replaceAll("\\]+", "\\]");

		String parsedQuery = "";

		String[] tokens = subquery.split("\\],\\["); 

		
			
		for(int count = 0; count < tokens.length; count++)
		{
			String current = tokens[count];
	
			if(count == 0)
			{
				current = current.replaceAll("\\s+","");
				current = current.substring(1, current.length() - 1);
				current = current + "]";
			}
			else if (count == tokens.length - 1)
			{
				current = "[" + current;
				current = current.substring(0, current.length() - 2);
			}
			else
			{
				current = "[" + current + "]";
			}

			System.out.println(current);
			parsedQuery += recursiveInterpreter(current);
		}
		

    		return parsedQuery.substring(0, parsedQuery.length() - 1) + ".";
  	}

	public static String recursiveInterpreter(String text)
  	{
		if(text.length() == 0) return "";
		String parameters = text.substring(1, text.length() - 2);

		String[] parameter_array = parameters.split(","); 

		String predicate = "";

		for(int count = 0; count < parameter_array.length; count++)
		{
			String current = parameter_array[count].replace("\\\"", "");
			current = current.replace(":", "");
			current = current.replace("variable", "");
			current = current.replace("\\", "");

			predicate += current;
			if(count == 0 && parameter_array.length > 1) predicate += "(";
			else if(count == parameter_array.length -1  && parameter_array.length > 1) predicate += "),";
			else predicate += ",";
		}
		return predicate;
	}

}
