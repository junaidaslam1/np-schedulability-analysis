#ifndef IO_HPP
#define IO_HPP

#include <iostream>
#include <utility>

#include "interval.hpp"
#include "time.hpp"
#include "jobs.hpp"
#include "precedence.hpp"
#include "aborts.hpp"
#include "config.h"

static uint32_t s_NumberOfNodeTypes = 0;

namespace NP {

	inline bool isStringEqual(char *s1, char *s2, int length) {
		for(int i=0;i<length;i++) {
			if(s1[i] != s2[i]) {
				return false;
			}
		} return true;
	}

	inline void skip_over(std::istream& in, char c)
	{
		while (in.good() && in.get() != (int) c)
			/* skip */;
	}

	inline bool skip_one(std::istream& in, char c)
	{
		if (in.good() && in.peek() == (int) c) {
			in.get(); /* skip */
			return true;
		} else
			return false;
	}

	inline void skip_all(std::istream& in, char c)
	{
		while (skip_one(in, c))
			/* skip */;
	}

	inline bool more_data(std::istream& in)
	{
		in.peek();
		return !in.eof();
	}

	inline void next_field(std::istream& in)
	{
		// eat up any trailing spaces
		skip_all(in, ' ');
		// eat up field separator
		skip_one(in, ',');
	}

	inline void next_line(std::istream& in)
	{
		skip_over(in, '\n');
	}

	inline JobID parse_job_id(std::istream& in)
	{
		unsigned long jid, tid;

		in >> tid;
		next_field(in);
		in >> jid;
		return JobID(jid, tid);
	}

	inline Precedence_constraint parse_precedence_constraint(std::istream &in)
	{
		std::ios_base::iostate state_before = in.exceptions();
		in.exceptions(std::istream::failbit | std::istream::badbit);

		// first two columns
		auto from = parse_job_id(in);

		next_field(in);

		// last two columns
		auto to = parse_job_id(in);

		in.exceptions(state_before);

		return Precedence_constraint(from, to);
	}

	inline Precedence_constraints parse_dag_file(std::istream& in)
	{
		Precedence_constraints edges;

		// skip column headers
		next_line(in);

		// parse all rows
		while (more_data(in)) {
			// each row contains one precedence constraint
			edges.push_back(parse_precedence_constraint(in));
			next_line(in);
		}

		return edges;
	}

	template<class Time> Job<Time> parse_job(std::istream& in, std::vector<uint32_t> *NodesOfEachComputingNodeType = 0, bool inheterogenous_analysis = false)
	{
		unsigned long tid, jid;

		unsigned int resource_type;

		std::ios_base::iostate state_before = in.exceptions();

		Time arr_min, arr_max, cost_min, cost_max, dl, prio;

		in.exceptions(std::istream::failbit | std::istream::badbit);

		in >> tid;
		next_field(in);
		in >> jid;
		next_field(in);
		in >> arr_min;
		next_field(in);
		in >> arr_max;
		next_field(in);
		in >> cost_min;
		next_field(in);
		in >> cost_max;
		next_field(in);
		in >> dl;
		next_field(in);
		in >> prio;

		// Check if the analysis is of heterogeneous type. if not then put a dummy number
		if(inheterogenous_analysis) {
			next_field(in);
			in >> resource_type;
			try {
				(*NodesOfEachComputingNodeType).at(resource_type);  // vector::at throws an out-of-range exception if not in range
			}
			catch (const std::out_of_range& oor) {
				std::cerr << "Out of Range Job resource_type error: " << oor.what() << " Check file format \n";
				exit(1);
			}
		} else {
			resource_type = DUMMY_RESOURCE_TYPE;
		}

		in.exceptions(state_before);

		return Job<Time>{jid, Interval<Time>{arr_min, arr_max},
						 Interval<Time>{cost_min, cost_max}, dl, prio, tid, resource_type};
	}

	static const uint32_t get_next_number_of_computing_nodes(std::istream& in, int Parse_Loop) {
		int lvNextNumberOfComputingNodes = 0;
		char lvChar = 0;

		for (int i = 0; i < Parse_Loop; i++) {
			skip_all(in, ' ');
			skip_all(in, ',');
			in>>lvChar;
			if(lvChar == '#') {
				lvNextNumberOfComputingNodes = DUMMY_RESOURCE_TYPE;
				return lvNextNumberOfComputingNodes;
			}
		}

		skip_all(in, ' ');
		skip_all(in, ',');
		in>>lvNextNumberOfComputingNodes;

		return lvNextNumberOfComputingNodes;
	}

	template<class Time>
	typename Job<Time>::Job_set	parse_file(
											std::istream& in,
											bool heterogeneous = false,
											uint32_t *outNumberOfComputingNodeTypes = 0,
											std::vector<uint32_t> *NodesOfEachComputingNodeType = 0,
											uint32_t *outNumberOfTotalResources = 0
										  )
	{
		typename Job<Time>::Job_set jobs;

		/* Start of Resource Exploration Parsing for Heterogeneous possibility */
		if(heterogeneous)
		{
			/* The file format is changed in this case.
			 * Step 1 : The first line now should contain information on number of computing node types (Z = ?)
			 * subsequent rows provide number of cores / computing nodes per computing node type which
			 * we obtained in first step
			 * resource type. For example, if Z = 4, there will be total of M0 to M3 rows,
			 * each with a number showing the cores available. M0 will be SUSPENSION_CORES since it always correspond to
			 * Suspension cores even if it used or not in the jobset.
			 * */
			// Checking Number of Resource Types.
			char ins[14];
			in.read(ins, 13);

			if(isStringEqual(&ins[0], (char*)"ResourceTypes", 13))
			{
				// Skip these characters of csv file
				skip_all(in, ' ');
				skip_one(in, ',');
				in>>*outNumberOfComputingNodeTypes;
				skip_all(in, ' ');
				skip_all(in, ',');
			}
			else
			{
				std::cerr<<"error: invalid file format"<<std::endl;
			}

			int Parse_Loop = 0;

			if(*outNumberOfComputingNodeTypes > 0) {
				s_NumberOfNodeTypes	=	*outNumberOfComputingNodeTypes;
				do {
					if(Parse_Loop < 10) {
						if(Parse_Loop == 0) {
							(void)get_next_number_of_computing_nodes(in, 2); // This statement is to just skip the M0 line
							(*NodesOfEachComputingNodeType).push_back(0);
						} else {
							(*NodesOfEachComputingNodeType).push_back(get_next_number_of_computing_nodes(in, 2));
						}
					} else if (Parse_Loop < 100) {
						(*NodesOfEachComputingNodeType).push_back(get_next_number_of_computing_nodes(in, 3));
					} else {
						(*NodesOfEachComputingNodeType).push_back(get_next_number_of_computing_nodes(in, 4));
					}
					Parse_Loop++;
					if((*NodesOfEachComputingNodeType)[Parse_Loop-1] == DUMMY_RESOURCE_TYPE) {
						(*NodesOfEachComputingNodeType).pop_back();
					} else {
						*outNumberOfTotalResources += (*NodesOfEachComputingNodeType)[Parse_Loop-1];
					}
				} while(((*NodesOfEachComputingNodeType)[Parse_Loop-1] != DUMMY_RESOURCE_TYPE) && (Parse_Loop <= *outNumberOfComputingNodeTypes));
				*outNumberOfTotalResources += 1; // Add one extra for index 0 of PA and CA which always correspond to self suspension type core
				// Skip the '#' Line
				next_line(in);
			}
		}
		else {
			char lvChar;
			in >> lvChar;
			if(lvChar != 'T') {
				while(lvChar != 'T') {
					next_line(in);
					in >> lvChar;
				}
			}
		}
		/* End of Resource Exploration Parsing for Heterogeneous possibility */

		next_line(in);

		while (more_data(in)) {
			jobs.push_back(parse_job<Time>(in, NodesOfEachComputingNodeType, heterogeneous));
			// munge any trailing whitespace or extra columns
			next_line(in);
		}

		return jobs;
	}

	template<class Time>
	Abort_action<Time> parse_abort_action(std::istream& in)
	{
		unsigned long tid, jid;
		Time trig_min, trig_max, cleanup_min, cleanup_max;

		std::ios_base::iostate state_before = in.exceptions();

		in.exceptions(std::istream::failbit | std::istream::badbit);

		in >> tid;
		next_field(in);
		in >> jid;
		next_field(in);
		in >> trig_min;
		next_field(in);
		in >> trig_max;
		next_field(in);
		in >> cleanup_min;
		next_field(in);
		in >> cleanup_max;

		in.exceptions(state_before);

		return Abort_action<Time>{JobID{jid, tid},
		                          Interval<Time>{trig_min, trig_max},
		                          Interval<Time>{cleanup_min, cleanup_max}};
	}


	template<class Time>
	std::vector<Abort_action<Time>> parse_abort_file(std::istream& in)
	{
		// first row contains a comment, just skip it
		next_line(in);

		std::vector<Abort_action<Time>> abort_actions;

		while (more_data(in)) {
			abort_actions.push_back(parse_abort_action<Time>(in));
			// munge any trailing whitespace or extra columns
			next_line(in);
		}

		return abort_actions;
	}


}

#endif
