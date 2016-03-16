/**
 * Knowrob Information Expoter from CNRS Robot Database
 * Done for SAPHARI Project 2nd year review in Toulouse, France 
 * 1/30/2014
 * 
 * @author Asil Kaan Bozcuoglu, asil@cs.uni-bremen.de
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of SAPHARI Project nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/package.h>
#include <json_prolog/prolog.h>
#include "std_msgs/String.h"
#include <boost/tokenizer.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace json_prolog;
using namespace boost;

Prolog* pl;
const string prefix = "http://www.saphari.eu/cnrs.owl#";

void databaseCallback(const std_msgs::String::ConstPtr& msg)
{
   ROS_INFO("Received: [%s]", msg->data.c_str());

   PrologQueryProxy bdgs_init = pl->query("rdf_retractall(A, knowrob:'near', B)");

  	for(PrologQueryProxy::iterator it=bdgs_init.begin();
      		it != bdgs_init.end(); it++)
  	{
     		cout << "--" << endl;
  	}

	// laas semantic map file
  	PrologQueryProxy bdgs_sm = pl->query("rdf_retractall(A, knowrob:'on-Physical', B)");

  	for(PrologQueryProxy::iterator it=bdgs_sm.begin();
      		it != bdgs_sm.end(); it++) 
  	{
     		cout << "--" << endl;
  	}   

   PrologQueryProxy bdgs_init2 = pl->query("rdf_retractall(A, knowrob:'in-Physical', B)");

  	for(PrologQueryProxy::iterator it=bdgs_init2.begin();
      		it != bdgs_init2.end(); it++)
  	{
     		cout << "--" << endl;
  	}

	// laas semantic map file
  	PrologQueryProxy bdgs_sm2 = pl->query("rdf_retractall(A, knowrob:'reachableFrom', B)");

  	for(PrologQueryProxy::iterator it=bdgs_sm2.begin();
      		it != bdgs_sm2.end(); it++) 
  	{
     		cout << "--" << endl;
  	}

   PrologQueryProxy bdgs_init3 = pl->query("rdf_retractall(A, knowrob:'visibleFrom', B)");

  	for(PrologQueryProxy::iterator it=bdgs_init3.begin();
      		it != bdgs_init3.end(); it++)
  	{
     		cout << "--" << endl;
  	}




   std::ofstream logging;

   logging.open("log.kno", std::ios_base::app);


   //int count = 0;
 
   typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
   boost::char_separator<char> sep(")");
   tokenizer tokens(msg->data, sep);
   //tokenizer::iterator tok_iter1 = tokens.end();
   //ROS_INFO("Received: [%s]", (*tok_iter1).c_str());

    


   for (tokenizer::iterator tok_iter = tokens.begin();
       tok_iter != tokens.end(); ++tok_iter)
   {
	//std::cout << "<" << *tok_iter << "> " << endl;
	logging << "<" << *tok_iter << "> " << endl;

	if(*tok_iter == " ." || *tok_iter == " (CHECKING-REPLY-QUEUE" || *tok_iter == " (START-AND-CHECK-REPLY-QUEUE-WITH-PRIORITY") break;


        boost::char_separator<char> inner_sep(" ");
   	tokenizer inner_tokens(*tok_iter, inner_sep);

	tokenizer::iterator inner_tok_cmd = inner_tokens.begin();
	tokenizer::iterator inner_tok_perspective = ++inner_tok_cmd;
	tokenizer::iterator inner_tok_object = ++inner_tok_cmd;
	tokenizer::iterator inner_tok_predicate = ++inner_tok_cmd;
	tokenizer::iterator inner_tok_object2 = ++inner_tok_cmd;
	tokenizer::iterator inner_tok_time = ++inner_tok_cmd;

	// std::cout << "\\" << *inner_tok_predicate << "\\" << endl;;
	if(*inner_tok_predicate == "isNextTo")
	{
		//cout << "rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg = pl->query("rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
			//cout << "the previous relation is deleted" << endl;
			cout << "---" << endl;


		cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'near', '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg_assert = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'near', '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg_assert.begin(); it != bdg_assert.end(); it++)
			//cout << "inserted" << endl;
			cout << "---" << endl;
		
	}
	else if(*inner_tok_predicate == "isOn")
	{
		//cout << "rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg = pl->query("rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
			//cout << "the previous relation is deleted" << endl;
			cout << "---" << endl;

		cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'on-Physical', '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg_assert = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'on-Physical', '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg_assert.begin(); it != bdg_assert.end(); it++)
			//cout << "inserted" << endl;
			cout << "---" << endl;

	}
	else if(*inner_tok_predicate == "isIn")
	{
		//cout << "rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg = pl->query("rdf_retractall('" + prefix + *inner_tok_object + "', A, '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
			//cout << "the previous relation is deleted" << endl;
			cout << "---" << endl;

		cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'in-Physical', '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg_assert = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'in-Physical', '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg_assert.begin(); it != bdg_assert.end(); it++)
			//cout << "inserted" << endl;
			cout << "---" << endl;

		PrologQueryProxy bdg_assert2 = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'on-Physical', '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg_assert2.begin(); it != bdg_assert2.end(); it++)
			cout << "---" << endl;

	}
	else if(*inner_tok_predicate == "isReachable")
	{
		if(*inner_tok_object2 == "true")
		{	
			cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'reachableFrom', '" + prefix + *inner_tok_perspective + "')" << endl;
			PrologQueryProxy bdg = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'reachableFrom', '" + prefix + *inner_tok_perspective + "')");

			for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
				//cout << "inserted" << endl;
				cout << "---" << endl;
		}
		else
		{
			//cout << "rdf_retractall('" + prefix + *inner_tok_object + "', knowrob:'reachableFrom', '" + prefix + *inner_tok_perspective + "')" << endl;
			PrologQueryProxy bdg = pl->query("rdf_retractall('" + prefix + *inner_tok_object + "', knowrob:'reachableFrom', '" + prefix + *inner_tok_perspective + "')");

			for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
				//cout << "deleted" << endl;
				cout << "---" << endl;
		}
	}
	else if(*inner_tok_predicate == "isVisibleBy" &&  *inner_tok_perspective == "PR2_ROBOT" && *inner_tok_object != "IKEA_SHELF_DARK")
	{

		cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_object2 + "')" << endl;
		PrologQueryProxy bdg_assert = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_object2 + "')");

		for(PrologQueryProxy::iterator it=bdg_assert.begin(); it != bdg_assert.end(); it++)
			//cout << "inserted" << endl;
			cout << "---" << endl;

		/*if(*inner_tok_object2 == "true")
		{
			//cout << "rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_perspective + "')" << endl;
			PrologQueryProxy bdg = pl->query("rdf_assert('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_perspective + "')");

			for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
				//cout << "inserted" << endl;
				cout << "---" << endl;
		}
		else
		{
			//cout << "rdf_retractall('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_perspective + "')" << endl;
			PrologQueryProxy bdg = pl->query("rdf_retractall('" + prefix + *inner_tok_object + "', knowrob:'visibleFrom', '" + prefix + *inner_tok_perspective + "')");

			for(PrologQueryProxy::iterator it=bdg.begin(); it != bdg.end(); it++)
				//cout << "deleted" << endl;
				cout << "---" << endl;
		}*/

	}

   }
   logging.close();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "knowrob_listener");
  	ros::NodeHandle n;

	pl = new Prolog;

	ros::Subscriber sub = n.subscribe("prs_message", 1000, databaseCallback);

	ros::service::waitForService("/json_prolog/query", -1);
  
  	PrologQueryProxy bdgs_init = pl->query("register_ros_package(ias_semantic_map)");

  	for(PrologQueryProxy::iterator it=bdgs_init.begin();
      		it != bdgs_init.end(); it++)
  	{
     		cout << "ias_semantic_map package loaded!" << endl;
  	}

	// laas semantic map file
  	PrologQueryProxy bdgs_sm = pl->query("owl_parser:owl_parse('/home/viki/asil-ros-pkg/bremen_toulouse/owl/cnrs.owl', false, false, true)");

  	for(PrologQueryProxy::iterator it=bdgs_sm.begin();
      		it != bdgs_sm.end(); it++) 
  	{
     		cout << "laas semantic map file loaded!" << endl;
  	}

	ros::spin();
	return 0;
}


