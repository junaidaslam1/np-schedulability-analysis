#include <iostream>
#include <ostream>
#include <cassert>
#include <algorithm>
#include <list>
#include <iterator>

#include <set>

#include "util.hpp"
#include "index_set.hpp"
#include "jobs.hpp"
#include "cache.hpp"
#include "config.h"

typedef struct{
	uint16_t Job_Index = MAX_JOBS_IN_HP;
	uint32_t EFT;
	uint32_t LFT;
}StateCache;

namespace NP {

	namespace Global {

		typedef std::size_t Job_index;
		typedef std::vector<Job_index> Job_precedence_set;
		typedef std::vector<uint32_t> NodesPerNodeType;
		typedef std::vector<uint32_t> ResourceIndex;

		template<class Time>
		class Schedule_state
		{
			public:

			Schedule_state(unsigned int num_processors, NodesPerNodeType inVector = {0})
			: scheduled_jobs()
			, num_jobs_scheduled(0)
			, core_avail{num_processors, Interval<Time>(Time(0), Time(0))}
			, lookup_key{0x9a9a9a9a9a9a9a9aUL}
			, nodes_per_node_type(inVector)
			{
				ResourceTypeBaseIndex.push_back(0);

#ifdef ENABLE_STATE_CACHING
				NumberOfCachedElements = 0;
#endif
				// construct base indexes for each of the resource types
				for(int i = 1; i < nodes_per_node_type.size(); i++)
				{
					uint32_t lvIndex = 0;
					for(int j=0; j<i; j++)
					{
						lvIndex += nodes_per_node_type[j];
					}
					lvIndex += 1;
					ResourceTypeBaseIndex.push_back(lvIndex);
				}

				resource_type = DUMMY_RESOURCE_TYPE;
				assert(core_avail.size() > 0);
			}

			Schedule_state
						  (
							const Schedule_state& from,
							Job_index j,
							const Job_precedence_set& predecessors,
							Interval<Time> start_times,
							Interval<Time> finish_times,
							hash_value_t key,
							unsigned int job_resource_type = DUMMY_RESOURCE_TYPE
						  )
			: num_jobs_scheduled(from.num_jobs_scheduled + 1)
			, scheduled_jobs{from.scheduled_jobs, j}
			, lookup_key{from.lookup_key ^ key}
			, resource_type(job_resource_type)
			, nodes_per_node_type(from.nodes_per_node_type)
			, ResourceTypeBaseIndex(from.ResourceTypeBaseIndex)
#ifdef ENABLE_STATE_CACHING
			, DispatchedJobCache(from.DispatchedJobCache)
			, NumberOfCachedElements(from.NumberOfCachedElements)
#endif
			{
				auto est = start_times.min();
				auto lst = start_times.max();
				auto eft = finish_times.min();
				auto lft = finish_times.max();

				DM("est: " << est << std::endl
				<< "lst: " << lst << std::endl
				<< "eft: " << eft << std::endl
				<< "lft: " << lft << std::endl);

				std::vector<Time> ca, pa;

				update_pa_ca(
							  from,
							  &pa, &ca,
							  ResourceTypeBaseIndex[(resource_type==DUMMY_RESOURCE_TYPE?0:resource_type)],
							  eft, lft, est,
							  resource_type
							);

				// update scheduled jobs
				// keep it sorted to make it easier to merge
				auto ca_it = ca.begin();
				if(resource_type != DUMMY_RESOURCE_TYPE) {
					ca_it = ca.begin()+1; // skipping ca[0] since that is for suspension type, hence all of the self suspension jobs
				}

				bool added_j = false;
				for (const auto& rj : from.certain_jobs) {
					auto x = rj.first;
					auto x_eft = rj.second.min();
					auto x_lft = rj.second.max();

					if (lst <= x_eft) {
						if (!added_j && rj.first > j) {
							// right place to add j
							certain_jobs.emplace_back(j, finish_times);
							added_j = true;
						}
						certain_jobs.emplace_back(rj);
					}
				}

				// if we didn't add it yet, add it at the back
				if (!added_j) {
					certain_jobs.emplace_back(j, finish_times);
				}

				// sort in non-decreasing order
				if(resource_type == DUMMY_RESOURCE_TYPE) { // homogeneous resource type
					std::sort(pa.begin(), pa.end());
					std::sort(ca.begin(), ca.end());
				} else { // heterogeneous resource type
					for(int i = 1; i < nodes_per_node_type.size(); i++) {
						std::sort(pa.begin()+ResourceTypeBaseIndex[i], pa.begin()+ResourceTypeBaseIndex[i]+nodes_per_node_type[i]);
						std::sort(ca.begin()+ResourceTypeBaseIndex[i], ca.begin()+ResourceTypeBaseIndex[i]+nodes_per_node_type[i]);
					}
				}

				for (int i = 0; i < from.core_avail.size(); i++)
				{
					DM(i << " -> " << pa[i] << ":" << ca[i] << std::endl);
					core_avail.emplace_back(pa[i], ca[i]);
				}

#ifdef ENABLE_STATE_CACHING
				update_job_cache((uint16_t)j, finish_times);
#endif
				assert(core_avail.size() > 0);
				DM("*** new state: constructed " << *this << std::endl);
			}

			void update_pa_ca(	const Schedule_state& from,
								std::vector<Time> *PA,
								std::vector<Time> *CA,
								unsigned int index,
								Time eft, Time lft,
								Time est, unsigned int inResourceType = DUMMY_RESOURCE_TYPE
							)
			{
				/* This function updates the core availability values
				 * for each core of each resource types in case of heterogeneous analysis.
				 * Similarly, for homogeneous analysis it updates core availability intervals
				 * which is compliant with the earlier versions.
				 * */
				if(inResourceType != DUMMY_RESOURCE_TYPE)
				{
					if(inResourceType == SELF_SUSPENSION_TYPE) {
						if(!est) {
							(*PA).push_back(from.core_avail[SELF_SUSPENSION_TYPE].min());
							(*CA).push_back(from.core_avail[SELF_SUSPENSION_TYPE].max());
						} else {
							(*PA).push_back(std::max(est, from.core_avail[SELF_SUSPENSION_TYPE].min()));
							(*CA).push_back(std::max(est, from.core_avail[SELF_SUSPENSION_TYPE].max()));
						}
					} else {
						(*PA).push_back(from.core_avail[SELF_SUSPENSION_TYPE].min());
						(*CA).push_back(from.core_avail[SELF_SUSPENSION_TYPE].max());
					}
				} else if (!index) { // homogeneous
					(*PA).push_back(eft);
					(*CA).push_back(lft);
				}

				// skip first element in from.core_avail
				// For the rest reorder the core type and rest of them simply push as it is
				for (int i = 1; i < from.core_avail.size(); i++)
				{
					if((index > 0) && (index == i)) { // if heterogeneous push at its base index
						(*PA).push_back(eft);
						(*CA).push_back(lft);
						i += 1; // increase i to match with index of j
						for(int j = index+1; j < (index+nodes_per_node_type[inResourceType]); j++)
						{
							(*PA).push_back(std::max(est, from.core_avail[j].min()));
							(*CA).push_back(std::max(est, from.core_avail[j].max()));
							i += 1; // increase i to match with index of j
						}
						// Since i is incremented in last step, push one here before proceeding
						if(i < from.core_avail.size())
						{
							(*PA).push_back(from.core_avail[i].min());
							(*CA).push_back(from.core_avail[i].max());
						}
					} else {
						if(inResourceType == DUMMY_RESOURCE_TYPE) { // homogeneous
							(*PA).push_back(std::max(est, from.core_avail[i].min()));
							(*CA).push_back(std::max(est, from.core_avail[i].max()));
						} else { // heterogeneous
							(*PA).push_back(from.core_avail[i].min());
							(*CA).push_back(from.core_avail[i].max());
						}
					}
				}
			}

#ifdef ENABLE_STATE_CACHING
			void Pop_Cache_Entry(void) {
				// check if cache is fully filled
				for(uint16_t i=0; i < (STATE_CACHE_SIZE-1); i++) {
					DispatchedJobCache[i] = DispatchedJobCache[i+1];
				}
			}

			void update_job_cache(uint16_t inJobIndexes, Interval<Time> inFinishTimes)
			{

				short J_Size = 1;

				for(int i = 0; i < J_Size; i++)
				{
					if(NumberOfCachedElements == STATE_CACHE_SIZE)
					{

						StateCache lvCache = {inJobIndexes, (uint32_t)inFinishTimes.from(), (uint32_t)inFinishTimes.until()};

						Pop_Cache_Entry();
						DispatchedJobCache[STATE_CACHE_SIZE - 1] = lvCache;

					} else {

						StateCache lvCache = {inJobIndexes, (uint32_t)inFinishTimes.from(), (uint32_t)inFinishTimes.until()};

						DispatchedJobCache[NumberOfCachedElements] = lvCache;
						NumberOfCachedElements++;
					}
				}

				std::sort(std::begin(DispatchedJobCache), std::end(DispatchedJobCache),
						[](const StateCache& x, const StateCache& y)
						{return (x.Job_Index < y.Job_Index);});
			}
#endif

			hash_value_t get_key() const
			{
				return lookup_key;
			}

			std::size_t get_size_scheduled_jobs(void) const
			{
				return scheduled_jobs.size();
			}

			bool same_jobs_scheduled(const Schedule_state &other) const
			{
				return scheduled_jobs == other.scheduled_jobs;
			}

			bool can_merge_with(const Schedule_state<Time>& other) const
			{
				assert(core_avail.size() == other.core_avail.size());

				if (get_key() != other.get_key())
					return false;
				if (!same_jobs_scheduled(other))
					return false;

				/* No need to change here JATW
				 * */
				for (int i = 0; i < core_avail.size(); i++) {
						if (!core_avail[i].intersects(other.core_avail[i])){
							return false;
						}
				}

				return true;
			}

			bool try_to_merge(const Schedule_state<Time>& other)
			{
				if (!can_merge_with(other))
					return false;

				/* No need to change here JATW
			    * */
				for (int i = 0; i < core_avail.size(); i++) {
					core_avail[i] |= other.core_avail[i];
				}

				// vector to collect joint certain jobs
				std::vector<std::pair<Job_index, Interval<Time>>> new_cj;

				// walk both sorted job lists to see if we find matches
				/* This is not to be changed since only those two jobs will
				 * merge which have the same index, consequently same resource type
				 * automatically.
				 * */
				auto it = certain_jobs.begin();
				auto jt = other.certain_jobs.begin();
				while (it != certain_jobs.end() &&
				       jt != other.certain_jobs.end()) {
					if (it->first == jt->first) {
						// same job
						new_cj.emplace_back(it->first, it->second | jt->second);
						it++;
						jt++;
					} else if (it->first < jt->first) {
						it++;
					}
					else {
						jt++;
					}
				}
				// move new certain jobs into the state
				certain_jobs.swap(new_cj);

#ifdef ENABLE_STATE_CACHING
				// Merging the cache of two states
				NumberOfCachedElements = 0;
				StateCache newDispatchedJobCache[STATE_CACHE_SIZE];
				for(uint16_t i = 0; i < STATE_CACHE_SIZE; i++) {
					for(uint16_t j = 0; j < STATE_CACHE_SIZE; j++) {
						if(
							(other.DispatchedJobCache[i].Job_Index == DispatchedJobCache[j].Job_Index) &&
							(other.DispatchedJobCache[i].Job_Index != MAX_JOBS_IN_HP)) // Make sure to not consider dummy job
						{
							StateCache lvCache = {	// construct cache entry
													other.DispatchedJobCache[i].Job_Index,
													(uint32_t)std::min(DispatchedJobCache[j].EFT, other.DispatchedJobCache[i].EFT),
													(uint32_t)std::max(DispatchedJobCache[j].LFT, other.DispatchedJobCache[i].LFT)
												 };
							newDispatchedJobCache[NumberOfCachedElements] = lvCache;
							NumberOfCachedElements++;
							break;
						}
					}
				}
				// update state cache with intersected entries
				for(uint16_t j = 0; j < STATE_CACHE_SIZE; j++) {
					DispatchedJobCache[j] = newDispatchedJobCache[j];
				}
#endif
				DM("+++ merged " << other << " into " << *this << std::endl);

				return true;
			}

			const unsigned int number_of_scheduled_jobs() const
			{
				return num_jobs_scheduled;
			}

			Interval<Time> core_availability(unsigned int inResourceType = DUMMY_RESOURCE_TYPE) const
			{
				assert(core_avail.size() > 0);				
				if(
					 (inResourceType == DUMMY_RESOURCE_TYPE) ||
					 (inResourceType == SELF_SUSPENSION_TYPE)
				   ) 
				{				
					return core_avail[0];
				} else {
					return core_avail[ResourceTypeBaseIndex[inResourceType]];
				}
			}

			bool get_finish_times(Job_index j, Interval<Time> &ftimes) const
			{
#ifdef ENABLE_STATE_CACHING
				// First check inside state cache
				for(uint16_t i = 0; i < STATE_CACHE_SIZE; i++) {
					if (j == DispatchedJobCache[i].Job_Index) {
						ftimes = Interval<Time>{(Time)DispatchedJobCache[i].EFT, (Time)DispatchedJobCache[i].LFT};
						return true;
					}
				}
#endif
				// Check Certainly Running Jobs
				for (const auto& rj : certain_jobs) {
					// check index
					if (j == rj.first) {
						ftimes = rj.second;
						return true;
					}
				/*	 Certain_jobs is sorted in order of increasing job index.
					 If we see something larger than 'j' we are not going
					 to find it. For large processor counts, it might make
					 sense to do a binary search instead.*/
					if (j < rj.first)
						return false;
				}
				return false;
			}

			const bool job_incomplete(Job_index j) const
			{
				return !scheduled_jobs.contains(j);
			}

			const bool job_ready(const Job_precedence_set& predecessors) const
			{
				for (auto j : predecessors)
					if (!scheduled_jobs.contains(j))
						return false;
				return true;
			}

			void print_scheduled_jobs(void) const {
				std::cout<<" Already_ScheduledJobs:"<<scheduled_jobs<<std::endl;
			}

			void print_certain_jobs(void) const {
				std::cout<<" CertainJobs: {";
				for(int i = 0; i<certain_jobs.size(); i++) {
					std::cout<<certain_jobs[i].first<<",";
				} std::cout<<"} StateKey:"<<(unsigned short)lookup_key<<std::endl;
			}

			friend std::ostream& operator<< (std::ostream& stream,
			                                 const Schedule_state<Time>& s)
			{
				stream << "Global::State(";
				for (const auto& a : s.core_avail)
					stream << "[" << a.from() << ", " << a.until() << "] ";
				stream << "(";
				for (const auto& rj : s.certain_jobs)
					stream << rj.first << "";
				stream << ") " << s.scheduled_jobs << ")";
				stream << " @ " << (unsigned short)s.get_key();
				return stream;
			}

			void print_vertex_label(std::ostream& out,
				const typename Job<Time>::Job_set& jobs) const
			{
				for (const auto& a : core_avail)
					out << "[" << a.from() << ", " << a.until() << "] ";
				out << "\\n";
				bool first = true;
				out << "{";
				for (const auto& rj : certain_jobs) {
					if (!first)
						out << ", ";
					out << "T" << jobs[rj.first].get_task_id()
					    << "J" << jobs[rj.first].get_job_id() << ":"
					    << rj.second.min() << "-" << rj.second.max();
					first = false;
				}
				out << "}";
			}

			private:

			const unsigned int num_jobs_scheduled;

			// set of jobs that have been dispatched (may still be running)
			Index_set scheduled_jobs;

			// imprecise set of certainly running jobs
			std::vector<std::pair<Job_index, Interval<Time>>> certain_jobs;

			// system availability intervals
			std::vector<Interval<Time>> core_avail;

#ifdef ENABLE_STATE_CACHING
			// This shows currently held valid cache entries
			uint16_t NumberOfCachedElements;
			// Max Cache structures
			StateCache DispatchedJobCache[STATE_CACHE_SIZE] = {0};
#endif
			hash_value_t lookup_key;

			// (4) Vector showing number of nodes per computing node type.
			NodesPerNodeType nodes_per_node_type;

			// resource_type of the dispatched job leading to this state
			unsigned int resource_type;

			// Base index of resource type to be used later
			ResourceIndex ResourceTypeBaseIndex;

 			// no accidental copies
			Schedule_state(const Schedule_state& origin)  = delete;
		};
	}
}
