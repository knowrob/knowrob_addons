/**
 * 
 * Done for SAPHARI Project 2nd year review in Toulouse, France 
 * 2/4/2014
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

using namespace std;
using namespace json_prolog;
using namespace boost;

Prolog* pl;
const string prefix = "http://www.saphari.eu/cnrs.owl#";


int main(int argc, char **argv)
{	
	ros::init(argc, argv, "knowrob_tester");
  	ros::NodeHandle n;

	pl = new Prolog;
	string command;
	string unbound;

	while(1)
	{
		cout << "Enter Prolog predicate:" << endl;
		cin >> command;
		
		cout << "Enter the first unbound variable to be shown:" << endl;
		cin >> unbound;

		cout << command << endl;  		
		PrologQueryProxy bdgs = pl->query(command);
		

		for(PrologQueryProxy::iterator it=bdgs.begin();
      			it != bdgs.end(); it++) 
  		{
			PrologBindings bdg = *it;
			cout << unbound << " = " << bdg[unbound].toString() << endl;
  		}
	}
	return 0;
}
