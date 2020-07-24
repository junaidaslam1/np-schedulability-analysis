#ifndef GLOBAL_SPACE_H
#define GLOBAL_SPACE_H

#include <unordered_map>
#include <map>
#include <vector>
#include <deque>
#include <forward_list>
#include <algorithm>

#include <iostream>
#include <ostream>
#include <cassert>

#include "config.h"

#ifdef CONFIG_PARALLEL
#include "tbb/concurrent_hash_map.h"
#include "tbb/enumerable_thread_specific.h"
#include "tbb/parallel_for.h"
#endif

#include "problem.hpp"
#include "clock.hpp"
#include "sys/sysinfo.h"

#include "global/state.hpp"

static bool is_heterogeneous = false;

static bool is_aborted = false;

namespace NP {

	namespace Global {

		template<class Time>
		class State_space
		{
			public:

			typedef Scheduling_problem<Time> Problem;
			typedef typename Scheduling_problem<Time>::Workload Workload;
			typedef Schedule_state<Time> State;
			typedef typename std::vector<uint32_t> NrOfNodesPerNodeType;

			static State_space explore(
					const Problem& prob,
					const Analysis_options& opts)
			{
				// doesn't yet support exploration after deadline miss
				assert(opts.early_exit);

				auto s = State_space(	prob.jobs,
										prob.dag,
										prob.num_processors,
										opts.timeout,
										opts.max_depth,
										opts.num_buckets,
										prob.nodes_per_node_type,
										opts.heterogeneous
									);

				s.be_naive = opts.be_naive;
				s.cpu_time.start();
				s.explore();
				s.cpu_time.stop();
				return s;
			}

			// convenience interface for tests
			static State_space explore_naively(
				const Workload& jobs,
				unsigned int num_cpus)
			{
				Problem p{jobs, num_cpus};
				Analysis_options o;
				o.be_naive = true;
				return explore(p, o);
			}

			// convenience interface for tests
			static State_space explore(
				const Workload& jobs,
				unsigned int num_cpus)
			{
				Problem p{jobs, num_cpus};
				Analysis_options o;
				return explore(p, o);
			}

			Interval<Time> get_finish_times(const Job<Time>& j) const
			{
				auto rbounds = rta.find(j.get_id());
				if (rbounds == rta.end()) {
					return Interval<Time>{0, Time_model::constants<Time>::infinity()};
				} else {
					return rbounds->second;
				}
			}

			bool is_schedulable() const
			{
				return !aborted;
			}

			bool was_timed_out() const
			{
				return timed_out;
			}

			unsigned long number_of_states() const
			{
				return num_states;
			}

			unsigned long number_of_edges() const
			{
				return num_edges;
			}

			unsigned long max_exploration_front_width() const
			{
				return width;
			}

			double get_cpu_time() const
			{
				return cpu_time;
			}

			// double ended queue of schedule states
			typedef std::deque<State> States;

			typedef std::vector<std::reference_wrapper<const Job<Time>>> Edged_Jobs;

#ifdef CONFIG_PARALLEL
			// Thread local storage of the double ended queue of schedule states
			typedef tbb::enumerable_thread_specific< States > Split_states;
			// Double ended queue of thread local storage variable split_states which contains schedule states per thread
			typedef std::deque<Split_states> States_storage;

#else
			typedef std::deque< States > States_storage;
#endif

#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH

			struct Edge {
				const Job<Time>* scheduled;
				const State* source;
				const State* target;
				const Interval<Time> finish_range;
				const unsigned short StateKey;
				const unsigned int RSC;
				Edge(const Job<Time>* s, const State* src, const State* tgt,
				     const Interval<Time>& fr, const unsigned int inRSC, const std::size_t inKey)
				: scheduled(s)
				, source(src)
				, target(tgt)
				, finish_range(fr)
				, RSC(inRSC)
				, StateKey((unsigned short)inKey)
				{
				}

				bool deadline_miss_possible() const
				{
					return scheduled->exceeds_deadline(finish_range.upto());
				}

				Time earliest_finish_time() const
				{
					return finish_range.from();
				}

				Time latest_finish_time() const
				{
					return finish_range.upto();
				}

				Time earliest_start_time() const
				{
					return finish_range.from() - scheduled->least_cost();
				}

				Time latest_start_time() const
				{
					return finish_range.upto() - scheduled->maximal_cost();
				}

			};

			const std::deque<Edge>& get_edges() const
			{
				return edges;
			}

			const States_storage& get_states() const
			{
				return states_storage;
			}

#endif
			private:

			// State_ref is an iterator/pointer element of a double ended queue of type State which is (Schedule_State<Time>)
			typedef typename std::deque<State>::iterator State_ref;
			// State_refs is a linked list of State_ref
			typedef typename std::forward_list<State_ref> State_refs;

#ifdef CONFIG_PARALLEL
			// States_Cache is a hashtable which can be accessed concurrently.
			// Each key maps to a linked list of iterators (State_ref) of type States(std::deque<State>)
			typedef tbb::concurrent_hash_map<hash_value_t, State_refs> States_Cache;

			// The accessor class is used by threads to concurrently access the hash map
			typedef typename States_Cache::accessor States_map_accessor;
#else
			typedef std::unordered_map<hash_value_t, State_refs> States_Cache;
			typedef typename std::deque<States_Cache>	States_map;
#endif

			// Job_ref is pointer to a single job
			typedef const Job<Time>* Job_ref;
			// Multimap is also a hash map but gives the flexibility to store multiple jobs with same key
			typedef std::multimap<Time, Job_ref> By_time_map;

			// not used
			typedef std::deque<State_ref> Todo_queue;

			/* Job_lut is a class containing a Bucket (Vector of Job references)
			 * A unique pointer 'buckets' to array of Bucket is defines.
			 * A range of intervals, width and total number buckets is also defined.
			 * At constructor time:
			 * - range of intervals is initialized by taking difference of maximum deadline of all the jobs with 0
			 * - Width is taken by dividing max_deadline of all jobs by number of buckets which is fixed 1000
			 */
			typedef Interval_lookup_table<Time, Job<Time>, Job<Time>::scheduling_window> Jobs_lut;

			typedef std::unordered_map<JobID, Interval<Time> > Response_times;

			bool heterogeneous = false;

			unsigned int PrevMinJobCount = 0, PrevMaxJobCount = 0;
#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH
			std::deque<Edge> edges;
#endif
			Response_times rta;

#ifdef CONFIG_PARALLEL
			tbb::enumerable_thread_specific<Response_times> partial_rta;
#endif

			bool aborted;
			bool timed_out;

			const unsigned int max_depth;

			bool be_naive;

			const Workload& jobs;

			// not touched after initialization
			std::vector<Job_precedence_set> _predecessors, _ancestors;
			// use these const references to ensure read-only access
			const std::vector<Job_precedence_set>& predecessors, &ancestors;
			typedef typename std::vector<By_time_map> By_time_map_vector;
			typedef typename std::vector<Jobs_lut> Jobs_lut_vector;

			Jobs_lut_vector	   _jobs_by_win;
			By_time_map_vector _jobs_by_latest_arrival;
			By_time_map_vector _jobs_by_earliest_arrival;
			By_time_map_vector _jobs_by_deadline;

			typename Jobs_lut_vector::const_iterator	jobs_by_win;
			typename By_time_map_vector::const_iterator jobs_by_latest_arrival;
			typename By_time_map_vector::const_iterator jobs_by_earliest_arrival;
			typename By_time_map_vector::const_iterator jobs_by_deadline;

			States_storage states_storage;

			States_Cache states_by_key;

			// updated only by main thread
			unsigned long num_states, width;
			unsigned long current_job_count;
			unsigned long num_edges;

			// (4) Vector showing number of nodes per computing node type.
			NrOfNodesPerNodeType nodes_per_node_type;

#ifdef CONFIG_PARALLEL
			tbb::enumerable_thread_specific<unsigned long> edge_counter;
#endif
			Processor_clock cpu_time;
			const double timeout;

			const unsigned int num_cpus;

			State_space(const Workload& jobs,
			            const Precedence_constraints &dag_edges,
			            unsigned int num_cpus,
			            double max_cpu_time = 0,
			            unsigned int max_depth = 0,
			            std::size_t num_buckets = 1000,
						NrOfNodesPerNodeType inVector = 0,
						bool AnalysisType = false)
			: jobs(jobs)
			, aborted(false)
			, timed_out(false)
			, be_naive(false)
			, timeout(max_cpu_time)
			, max_depth(max_depth)
			, num_states(0)
			, num_edges(0)
			, width(0)
			, current_job_count(0)
			, num_cpus(num_cpus)
			, _predecessors(jobs.size())
			, _ancestors(jobs.size())
			, predecessors(_predecessors)
			, ancestors(_ancestors)
			, nodes_per_node_type(inVector)
			, heterogeneous(AnalysisType)
			{
				is_heterogeneous = AnalysisType;

				do {
					_jobs_by_latest_arrival.emplace_back();
					_jobs_by_earliest_arrival.emplace_back();
					_jobs_by_deadline.emplace_back();
					_jobs_by_win.emplace_back(Interval<Time>{0, max_deadline(jobs)}, // Initialized jobs by scheduling window
							   max_deadline(jobs) / num_buckets);
				}while((_jobs_by_latest_arrival.size() != nodes_per_node_type.size()) && (nodes_per_node_type.size()>0));

				jobs_by_latest_arrival = _jobs_by_latest_arrival.begin();
				jobs_by_earliest_arrival = _jobs_by_earliest_arrival.begin();
				jobs_by_deadline = _jobs_by_deadline.begin();
				jobs_by_win = _jobs_by_win.begin();

				if(heterogeneous)
				{
					for (const Job<Time>& j : jobs)
					{
						_jobs_by_latest_arrival[j.get_resource_type()].insert({j.latest_arrival(), &j});
						_jobs_by_earliest_arrival[j.get_resource_type()].insert({j.earliest_arrival(), &j});
						_jobs_by_deadline[j.get_resource_type()].insert({j.get_deadline(), &j});
						_jobs_by_win[j.get_resource_type()].insert(j);
					}
				}
				else
				{
					nodes_per_node_type.push_back(0);

					for (const Job<Time>& j : jobs)
					{
						// Below is normal initialization
						_jobs_by_latest_arrival[0].insert({j.latest_arrival(), &j});
						_jobs_by_earliest_arrival[0].insert({j.earliest_arrival(), &j});
						_jobs_by_deadline[0].insert({j.get_deadline(), &j});
						_jobs_by_win[0].insert(j);
					}
				}
				/* Below for loop creates a 2D vector of jobs,
				 * which specify the order of their execution
				 * respecting their precedence constraints.
				 * */
				for (auto e : dag_edges)
				{
					const Job<Time>& from = lookup<Time>(jobs, e.first);
					const Job<Time>& to   = lookup<Time>(jobs, e.second);
					_predecessors[index_of(to)].push_back(index_of(from));
				}
				/*Below functionality creates ancestors of each job index*/
				for(unsigned int j = 0; j < jobs.size(); j++) {
					if(predecessors[j].size() > 0) {
						update_ancestors(j, predecessors, &_ancestors, j);
					}
					std::sort(_ancestors[j].begin(), _ancestors[j].end(), [](const Job_index& J1, const Job_index& J2){return (J1 < J2);});
				}
			}

			private:
			void count_edge()
			{
#ifdef CONFIG_PARALLEL
				edge_counter.local()++;
#else
				num_edges++;
#endif
			}
			void update_ancestors(
									std::size_t Job_Index,
									const std::vector<Job_precedence_set>& predecessors,
									std::vector<Job_precedence_set> *ancestors,
									std::size_t Pred_Index = 0
								 )
			{
				/* This is an Iterative function which creates an ancestors
				 * list for each of the jobs.
				 * */
				for(uint16_t p = 0; p < predecessors[Pred_Index].size(); p++) {
					bool PredExists = false;
					for(unsigned int i=0; i < (*ancestors)[Job_Index].size(); i++) {
						if((*ancestors)[Job_Index][i] == predecessors[Pred_Index][p]) {
							PredExists = true;
							break;
						}
					}
					if(!PredExists) {
						(*ancestors)[Job_Index].push_back(predecessors[Pred_Index][p]);
					}
					if(predecessors[predecessors[Pred_Index][p]].size() > 0) {
						update_ancestors(Job_Index, predecessors, ancestors, predecessors[Pred_Index][p]);
					}
				}
			}

			static Time max_deadline(const Workload &jobs)
			{
				Time dl = 0;
				for (const auto& j : jobs)
					dl = std::max(dl, j.get_deadline());
				return dl;
			}

			void update_finish_times(Response_times& r, const JobID& id,
			                         Interval<Time> range)
			{
				auto rbounds = r.find(id);
				if (rbounds == r.end()) {
					r.emplace(id, range);
				} else {
					rbounds->second |= range;
				}
				DM("RTA " << id << ": " << r.find(id)->second << std::endl);
			}

			void update_finish_times(
				Response_times& r, const Job<Time>& j, Interval<Time> range)
			{
				update_finish_times(r, j.get_id(), range);
				if (j.exceeds_deadline(range.upto()))
				{
					aborted = true;
				}
			}

			void update_finish_times(const Job<Time>& j, Interval<Time> range)
			{
				Response_times& r =
#ifdef CONFIG_PARALLEL
					partial_rta.local();
#else
					rta;
#endif
				update_finish_times(r, j, range);
			}


			std::size_t index_of(const Job<Time>& j) const
			{
				return (std::size_t) (&j - &(jobs[0]));
			}

			const Job_precedence_set& predecessors_of(const Job<Time>& j) const
			{
				/* index of the first vector which is base
				 * address of predecessors of that index.
				 * For example predecessors[i] contains std::vector<Job_index>
				 * which are its predecessors.
				 *  */
				return predecessors[index_of(j)];
			}

			void check_for_deadline_misses(const State& old_s, const State& new_s, unsigned int inResourceType = DUMMY_RESOURCE_TYPE)
			{
				unsigned int resource_type = inResourceType;
				auto check_from = old_s.core_availability(resource_type).min();
				auto earliest   = new_s.core_availability(resource_type).min();

				if(resource_type == DUMMY_RESOURCE_TYPE) {
					resource_type = 0;
				}

				// check if we skipped any jobs that are now guaranteed
				// to miss their deadline tied to specific resource types
				for (auto it = jobs_by_deadline[resource_type].lower_bound(check_from);
				     it != jobs_by_deadline[resource_type].end(); it++) {
					const Job<Time>& j = *(it->second);
					if ((j.get_deadline() < earliest) && (inResourceType != SELF_SUSPENSION_TYPE)) {
						if (unfinished(new_s, j)) {
							DM("deadline miss: " << new_s << " -> " << j << std::endl);
							// This job is still incomplete but has no chance
							// of being scheduled before its deadline anymore.
							// Abort.
							aborted = true;

							// create a dummy state for explanation purposes
							auto frange = new_s.core_availability(j.get_resource_type()) + j.get_cost();

							const State& next =
										new_state(new_s, index_of(j), predecessors_of(j),
												  frange, frange, j.get_key(), j.get_resource_type());
							// update response times
							update_finish_times(j, frange);
#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH
							edges.emplace_back(&j, &new_s, &next, frange, j.get_resource_type(), new_s.get_key());
#endif
							count_edge();
							break;

						}
					} else if ((inResourceType == SELF_SUSPENSION_TYPE) &&
							(j.get_deadline() < (earliest_ready_time(new_s, j) + j.get_cost().min()))
						   )
					{
						if (unfinished(new_s, j)) {

							aborted = true;

							auto frange = ready_times(new_s, j) + j.get_cost();

							const State& next =
										new_state(new_s, index_of(j), predecessors_of(j),
												  frange, frange, j.get_key(), j.get_resource_type());
							// update response times
							update_finish_times(j, frange);
#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH
								edges.emplace_back(&j, &new_s, &next, frange, j.get_resource_type(), new_s.get_key());
#endif
							count_edge();
							break;
						}
					} else {
						// deadlines now after the next earliest finish time
						break;
					}
				}
			}

			void make_initial_state()
			{
				// construct initial state
				states_storage.emplace_back();
				new_state(num_cpus, nodes_per_node_type);
			}

			States& states()
			{
#ifdef CONFIG_PARALLEL
			/* This statement extracts last element of a deque whose each element,
			 * (which is a deque of schedule states) is TLS specific.
			 * The format of states_storage is like this:
			 * Deque(states_storage)   (TLS   (States))		States
			 * [1]					  Deque_1	--->	1, 2, 3, 4, ...., s
			 * [2]					  Deque_2	--->	1, 2, 3, 4, ...., s
			 * [3]					  Deque_3   --->	1, 2, 3, 4, ...., s
			 * .		  				.		....	...................
			 * . 		  				.		....	...................
			 * .		  				.		....	...................
			 * [t]					  Deque_n	--->	1, 2, 3, 4, ...., s
			 *
			 * states_storage.back() returns Deque_n since its the last element of Deque
			 * (which is element 't'). Since states_storage.back().local() is called, so it
			 * returns TLS of Deque_n which is specific to each of the threads.
			 * Any reference to ('deque<State> States' (which is return type of this function))
			 * will be TLS and all of the container operations will be in context to TLS instance.
			 *
			 * (Deque<State> States) (State = Schedule_State<Time>)
			 * */
				return states_storage.back().local();
#else
				return states_storage.back();
#endif
			}

			template <typename... Args>
			State_ref alloc_state(Args&&... args)
			{
				states().emplace_back(std::forward<Args>(args)...);
				/* Below statement returns the last element (Schedule_State<Time>) of the Deque,
				 * which is obtained via call to states()
				 *
				 * State_ref is an iterator to elements of States(a deque of type Schedule_state<Time>)
				 * */
				State_ref s = --states().end();
				// make sure we didn't screw up...
				auto njobs = s->number_of_scheduled_jobs();
				assert (
					(!njobs && num_states == 0) // initial state
				    || (njobs == current_job_count + 1) // normal State
				    || (njobs == current_job_count + 2 && aborted) // deadline miss
				);

				return s;
			}

			void dealloc_state(State_ref s)
			{
				assert(--states().end() == s);
				states().pop_back();
			}

			template <typename... Args>
			State& new_state(Args&&... args)
			{
				return *alloc_state(std::forward<Args>(args)...);
			}

			template <typename... Args>
			State& new_or_merged_state(Args&&... args)
			{
				State_ref s_ref = alloc_state(std::forward<Args>(args)...);

				// try to merge the new state into an existing state
				State_ref s = merge_or_cache(s_ref);
				if (s != s_ref) {
					// great, we merged!
					// clean up the just-created state that we no longer need
					dealloc_state(s_ref);
				}
				return *s;
			}

#ifdef CONFIG_PARALLEL

			// make state available for fast lookup
			void insert_cache_state(States_map_accessor &acc, State_ref s)
			{
				assert(!acc.empty());

				State_refs& list = acc->second;
				list.push_front(s);
			}

			// returns true if state was merged
			State_ref merge_or_cache(State_ref s)
			{
				States_map_accessor acc;
				bool found_in_previous_depth_cache = false;

				while (true) {
					// check if key exists
					if (states_by_key.find(acc, s->get_key())) {
						for (State_ref other : acc->second) {
							if (other->try_to_merge(*s)) {
	//							std::cout<<"Merged::"<<std::endl;
								return other;
							}
						}
						// If we reach here, we failed to merge, so go ahead
						// and insert it. s is an iterator to 'States' (std::deque<State>)
						insert_cache_state(acc, s);
						return s;
					// otherwise, key doesn't exist yet, let's try to create it
					} else if (states_by_key.insert(acc, s->get_key())) {
						// We created the list, so go ahead and insert our state.
						insert_cache_state(acc, s);
						return s;
					}
					// if we raced with concurrent creation, try again
				}
			}

#else

			void cache_state(State_ref s)
			{
				// create a new list if needed, or lookup if already existing
				auto res = states_by_key.emplace(
					std::make_pair(s->get_key(), State_refs()));

				auto pair_it = res.first;
				State_refs& list = pair_it->second;

				list.push_front(s);
			}

			State_ref merge_or_cache(State_ref s_ref)
			{
				State& s = *s_ref;

				const auto pair_it = states_by_key.find(s.get_key());
				// cannot merge if key doesn't exist
				if (pair_it != states_by_key.end()) {
					for (State_ref other : pair_it->second) {
						if (other->try_to_merge(*s_ref)) {
//							std::cout<<"Merged::"<<std::endl;
							return other;
						}
					}
				}
				// if we reach here, we failed to merge
				cache_state(s_ref);
				return s_ref;
			}
#endif


			void check_cpu_timeout()
			{
				if (timeout && get_cpu_time() > timeout) {
					aborted = true;
					timed_out = true;
				}
			}

			void check_depth_abort()
			{
				if (max_depth && current_job_count > max_depth) {
					aborted = true;
				}
			}

			bool unfinished(const State& s, const Job<Time>& j) const
			{
				return s.job_incomplete(index_of(j));
			}

			bool ready(const State& s, const Job<Time>& j) const
			{
				return unfinished(s, j) && s.job_ready(predecessors_of(j));
			}

			bool all_jobs_scheduled(const State& s) const
			{
				return s.number_of_scheduled_jobs() == jobs.size();
			}

			// assumes j is ready
			Interval<Time> ready_times(const State& s, const Job<Time>& j) const
			{
				Interval<Time> r = j.arrival_window();
				for (auto pred : predecessors_of(j))
				{
					Interval<Time> ft{0, 0};
					if (!s.get_finish_times(pred, ft))
						ft = get_finish_times(jobs[pred]);
					r.lower_bound(ft.min());
					r.extend_to(ft.max());
				}
				return r;
			}

			// assumes j is ready
			Interval<Time> ready_times(
				const State& s, const Job<Time>& j,
				const Job_precedence_set& disregard) const
			{
				Interval<Time> r = j.arrival_window();
				for (auto pred : predecessors_of(j)) {
					// skip if part of disregard
					if (contains(disregard, pred))
						continue;
					Interval<Time> ft{0, 0};
					if (!s.get_finish_times(pred, ft))
						ft = get_finish_times(jobs[pred]);
					r.lower_bound(ft.min());
					r.extend_to(ft.max());
				}
				return r;
			}
#ifndef PESSIMISM_REDUCTION
			Time latest_ready_time(const State& s, const Job<Time>& j) const
#else
			Time latest_ready_time(const State& s, const Job<Time>& j, const Job<Time>& Ji) const
#endif
			{
#ifdef PESSIMISM_REDUCTION
				if((predecessors_of(j).size() == 1) && contains(ancestors[index_of(Ji)], predecessors_of(j)[0])) {
					return ready_times(s, j).min(); // max{rx^min, EFTp} since Ji and J has common ancestor predecessor
				}
#endif
				return ready_times(s, j).max();
			}

			Time earliest_ready_time(const State& s, const Job<Time>& j) const
			{
				return ready_times(s, j).min();
			}

			Time latest_ready_time(
				const State& s, Time earliest_ref_ready,
				const Job<Time>& j_hp, const Job<Time>& j_ref) const
			{
				auto rt = ready_times(s, j_hp, predecessors_of(j_ref));
#ifdef PESSIMISM_REDUCTION
				if((predecessors_of(j_hp).size() == 1) && contains(ancestors[index_of(j_ref)], predecessors_of(j_hp)[0])) {
					return std::max(rt.min(), earliest_ref_ready); // max{rx^min, EFTp} since Ji and Jhp has common ancestor predecessor
				}
#endif
				return std::max(rt.max(), earliest_ref_ready);

			}

			// Find next time by which any job is certainly released.
			// Note that this time may be in the past.
			Time next_higher_prio_job_ready(
				const State& s,
				const Job<Time> &reference_job,
				const Time t_earliest,
				Time *t_high_i = 0) const
			{
				unsigned int resource_type = reference_job.get_resource_type();
				auto ready_min = earliest_ready_time(s, reference_job);
				Time when = Time_model::constants<Time>::infinity();
				unsigned int lvSize_ResourceTypes = 0;

				if(resource_type == DUMMY_RESOURCE_TYPE) {
					resource_type = 0;
					lvSize_ResourceTypes = 1;
				} else {
					lvSize_ResourceTypes = nodes_per_node_type.size();
				}

				for(int i = 0; i < lvSize_ResourceTypes; i++) {
					auto Ai_Max = s.core_availability(i).max();
					for (const Job<Time>& j : jobs_by_win[i].lookup(t_earliest)) {
						if (ready(s, j)
							&& j.higher_priority_than(reference_job)) {
							if(resource_type == i) { // Homogeneous or same resource type in case of heterogeneous
								*t_high_i = std::min(when,latest_ready_time(s, ready_min, j, reference_job));
								when = *t_high_i;
							} else {	// Different Resource Types (Heterogeneous)
								auto t_high = std::max(
														latest_ready_time(s, ready_min, j, reference_job),
														Ai_Max
													  );
								when = std::min(when, t_high);
							}
						}
					}
				}
				// No point looking in the future when we've already
				// found one in the present.
				if (when <= t_earliest)
					return when;

				for(int i = 0; i < lvSize_ResourceTypes; i++)
				{
					auto Ai_Max = s.core_availability(i).max();
					// Ok, let's look also in the future.
					for (auto it = jobs_by_latest_arrival[i]
								   .lower_bound(t_earliest);
						 it != jobs_by_latest_arrival[i].end(); it++)
					{
						const Job<Time>& j = *(it->second);

						// check if we can stop looking
						if (when < j.latest_arrival())
							break; // yep, nothing can lower 'when' at this point

						// j is not relevant if it is already scheduled or blocked
						if (ready(s, j)
							&& j.higher_priority_than(reference_job)) {
							// does it beat what we've already seen?
							if(resource_type == i) { // Homogeneous or (same resource type in case of heterogeneous)
								*t_high_i = std::min(when,latest_ready_time(s, ready_min, j, reference_job));
								when = *t_high_i;
							} else { // Different Resource Types (Heterogeneous)
								auto t_high = std::max(
														latest_ready_time(s, ready_min, j, reference_job),
														Ai_Max
													  );
								when = std::min(when, t_high);
							}
						}
					}
				}
				return when;
			}

			// Find next time by which any job is certainly released.
			// Note that this time may be in the past.
#ifndef PESSIMISM_REDUCTION
			Time next_job_ready(const State& s, const Time t_earliest, unsigned int resource_type) const
#else
			Time next_job_ready(const State& s, const Time t_earliest, unsigned int resource_type, const Job<Time>& Ji) const
#endif
			{
				Time when = Time_model::constants<Time>::infinity();
				unsigned int lvResourceType = resource_type;
				if(lvResourceType == DUMMY_RESOURCE_TYPE) {
					lvResourceType = 0; // since plain multiprocessor has all jobs in 0 index of all vectors
				}
				// check everything that overlaps with t_earliest

				for (const Job<Time>& j : jobs_by_win[lvResourceType].lookup(t_earliest)) {
					if (ready(s, j)) {
#ifdef PESSIMISM_REDUCTION
						when = std::min(when, latest_ready_time(s, j, Ji));
#else
						when = std::min(when, latest_ready_time(s, j));
#endif
					}
				}
				// No point looking in the future when we've already
				// found one in the present.
				if (when <= t_earliest)
					return when;

				// Ok, let's look also in the future.
				for (auto it = jobs_by_latest_arrival[lvResourceType]
				               .lower_bound(t_earliest);
				     it != jobs_by_latest_arrival[lvResourceType].end(); it++)
				{
					const Job<Time>& j = *(it->second);
					// check if we can stop looking
					if (when < j.latest_arrival())
						break; // yep, nothing can lower 'when' at this point
					// j is not relevant if it is already scheduled or blocked
					if (ready(s, j)) {
						// does it beat what we've already seen?
#ifdef PESSIMISM_REDUCTION
						when = std::min(when, latest_ready_time(s, j, Ji));
#else
						when = std::min(when, latest_ready_time(s, j));
#endif
					}
				}

				return when;
			}

			// assumes j is ready
			// NOTE: we don't use Interval<Time> here because the Interval c'tor sorts its arguments.
			std::pair<Time, Time> start_times(
												const State& s,
												const Job<Time>& j,
												Time t_wc
											) const
			{
				auto rt = ready_times(s, j);
				auto at = s.core_availability(j.get_resource_type());

				Time est = 0, lst = 0, temp = 0;

				if(j.get_resource_type() == SELF_SUSPENSION_TYPE) {
					est = rt.min();
				} else {
					est = std::max(rt.min(), at.min());
				}

				DM("rt: " << rt << std::endl
				<< "at: " << at << std::endl);

				Time t_high_i = Time_model::constants<Time>::infinity();

				auto t_high = next_higher_prio_job_ready(s, j, at.min(), &t_high_i);

				lst = std::min(t_wc, t_high - Time_model::constants<Time>::epsilon());

				if(j.get_resource_type() == SELF_SUSPENSION_TYPE) {
					if(est <= t_wc) {
						lst = rt.max();
					} else {
						lst = t_wc;
					}
				}

				DM("est: " << est << std::endl);
				DM("lst: " << lst << std::endl);

				return {est, lst};
			}

			bool dispatch(
							const State& s,
							const Job<Time>& j,
							Time t_wc
						)
			{
				// check if this job has a feasible start-time interval
				auto _st = start_times(s, j, t_wc);
				if (_st.first > _st.second) {
					return false; // nope
				}

				Interval<Time> st{_st};

				// yep, job j is a feasible successor in state s
				// compute range of possible finish times
				Interval<Time> ftimes = st + j.get_cost();

				// update finish-time estimates
				update_finish_times(j, ftimes);

				// expand the graph, merging if possible
				const State& next = be_naive ?
					new_state(s, index_of(j), predecessors_of(j),
							  st, ftimes, j.get_key(), j.get_resource_type()) :
					new_or_merged_state(s, index_of(j), predecessors_of(j),
										st, ftimes, j.get_key(), j.get_resource_type());

				// make sure we didn't skip any jobs
				check_for_deadline_misses(s, next);

				#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH
					edges.emplace_back(&j, &s, &next, ftimes, j.get_resource_type(), next.get_key());
				#endif

				count_edge();
				return true;
			}

			bool new_states_exploration(
										const State& s,
										Time						 t_min						=	0,
										Time 						 t_wc						=	0,
										unsigned int 				 resource_type				=	DUMMY_RESOURCE_TYPE
									   )
			{
				bool found_one = false;
				bool first_job = true;

				DM(s << std::endl);
				DM("t_min: " << t_min << std::endl
				<< "t_wc: " << t_wc << std::endl);

				DM("==== [1] ====" << std::endl);
				// (1) first check jobs that may be already pending

				unsigned int lvResourceType = resource_type;
				if(lvResourceType == DUMMY_RESOURCE_TYPE) {
					lvResourceType = 0;
				}

				for (const Job<Time>& j : jobs_by_win[lvResourceType].lookup(t_min))
				{
					Interval<Time> st(0, 0), ft(0, 0);

					if (((j.earliest_arrival() <= t_min) && ready(s, j)))
					{
						bool    job_Found = false;
						Time est_other = 0, lst_other = 0;
#ifdef PESSIMISM_REDUCTION
						// When reduction in pessimism is required, the per job t_wc is calculated
						Time t_wc = Time_model::constants<Time>::infinity();
						t_wc = get_t_wc(s, j);
#endif
						job_Found = dispatch(s, j, t_wc);
						found_one |= job_Found;
					}
				}

				DM("==== [2] ====" << std::endl);

				// (2) check jobs that are released only later in the interval

				for (auto it = jobs_by_earliest_arrival[lvResourceType].upper_bound(t_min);
					 it != jobs_by_earliest_arrival[lvResourceType].end();
					 it++)
				{
					const Job<Time>& j = *it->second;
					DM(j << " (" << index_of(j) << ")" << std::endl);
					// stop looking once we've left the window of interest
#ifdef PESSIMISM_REDUCTION
					Time t_wc = Time_model::constants<Time>::infinity();
					t_wc = get_t_wc(s, j);
#endif
					if (j.earliest_arrival() > t_wc) {
						break;
					}

					assert(unfinished(s, j));

					if(ready(s, j))
					{
						bool    job_Found = false;
						Time est_other = 0, lst_other = 0;
						Interval<Time> st(0, 0), ft(0, 0);

						job_Found = dispatch(s, j, t_wc);
						found_one |= job_Found;
					}
				}
				return found_one;
			}

#ifdef PESSIMISM_REDUCTION
			Time get_t_wc(const State& s, const Job<Time>& j)
			{
				Time t_wc = Time_model::constants<Time>::infinity();
				// i = 0 is to consider homogeneous analysis.
				for(unsigned int i=0; i<nodes_per_node_type.size();i++)
				{
					// latest time some unfinished job is certainly ready
					auto Rj_max  = next_job_ready(s, s.core_availability(i).min(), i, j);
					// latest time some core is certainly available
					auto A1_max  = s.core_availability(i).max();
					// latest time by which a work-conserving scheduler
					// certainly schedules some job
					auto lv_twc = std::max(A1_max, Rj_max);

					if(lv_twc < t_wc) { // make sure this is lowest out of all resource types
						t_wc = lv_twc;
					}
				}
				return t_wc;
			}
#endif
			void explore(const State& s)
			{
				bool found_one = false;
				/* Following Data Structures shall be used
				 * for the creation of each new state
				 */
				DM("----" << std::endl);
				// (0) define the window of interest

				if(!heterogeneous)
				{
					Time t_wc = 0, t_min = 0;
					// First Get for the first resource type
					// earliest time a core is possibly available
#ifndef PESSIMISM_REDUCTION
					t_min  = s.core_availability(DUMMY_RESOURCE_TYPE).min();
					// latest time some unfinished job is certainly ready
					auto Rj_max  = next_job_ready(s, t_min, DUMMY_RESOURCE_TYPE);
					// latest time some core is certainly available
					auto A1_max = s.core_availability(DUMMY_RESOURCE_TYPE).max();
					// latest time by which a work-conserving scheduler
					// certainly schedules some job
					t_wc = std::max(A1_max, Rj_max);
#endif
					found_one = new_states_exploration(s, t_min, t_wc, DUMMY_RESOURCE_TYPE);
				}
				else
				{
					std::vector<Time> t_wc_i;

					t_wc_i.push_back(Time_model::constants<Time>::infinity());

					Time t_wc = Time_model::constants<Time>::infinity();

					for(unsigned int i=1; i<nodes_per_node_type.size();i++) // Excluding self suspension
					{
#ifndef PESSIMISM_REDUCTION
						// latest time some unfinished job is certainly ready
						auto Rj_max  = next_job_ready(s, s.core_availability(i).min(), i);
						// latest time some core is certainly available
						auto A1_max  = s.core_availability(i).max();
						// latest time by which a work-conserving scheduler
						// certainly schedules some job
						auto lv_twc = std::max(A1_max, Rj_max);

						t_wc_i.push_back(lv_twc);

						if(lv_twc < t_wc) { // make sure this is lowest out of all resource types
							t_wc = lv_twc;
						}
#endif
						t_wc_i.push_back(Time_model::constants<Time>::infinity());
					}

					for(unsigned int i=0; i<nodes_per_node_type.size();i++)
					{
						if(new_states_exploration(
													s, s.core_availability(i).min(),
													t_wc, i
												 )
						  )
						{
							found_one = true;
						}
					}
				}

				if (!found_one && !all_jobs_scheduled(s)) {
					// out of options and we didn't schedule all jobs
					aborted = true;
				}
			}

			// naive: no state merging
			void explore_naively()
			{
				be_naive = true;
				explore();
			}

			void explore()
			{
				/* Below statement creates the first element of deque states_storage
				 * (created element is another deque of 'States') which is in compliance with thread local storage
				 * Then creates first element in (thead local storage) 'States' with num_cpus [states.hpp -> line 26]
				 * */
				make_initial_state();
				std::size_t JobsSize = jobs.size();
				while (current_job_count < JobsSize)
				{
					unsigned long n;
#ifdef CONFIG_PARALLEL
					// new_states_part Gets the reference to last element
					const auto& new_states_part = states_storage.back();
					n = 0;

					for (const auto& new_states : new_states_part)
					{
						/* Why for loop? Does it traverses
						 * through all instances of TLS?
						 * Answer: Yes it goes through all TLS of new_states_part
						 * */
						n += new_states.size();
					}
#else
					n = 0;
					States& exploration_front = states();
					n = exploration_front.size();
#endif

					// allocate states space for next depth
					states_storage.emplace_back();

					// keep track of exploration front width
					width = std::max(width, n);

					num_states += n;

					check_depth_abort();
					check_cpu_timeout();

					if (aborted){
						break;
					}

#ifdef CONFIG_PARALLEL
					// This loop creates threads to divide the exploration work from the previously explored states
					parallel_for(new_states_part.range(),
					[&] (typename Split_states::const_range_type& r)
					{
						for (auto it = r.begin(); it != r.end(); it++)
						{
							const States& new_states = *it;
							auto s = new_states.size();
							tbb::parallel_for(tbb::blocked_range<size_t>(0, s),
								[&] (const tbb::blocked_range<size_t>& r) {
									for (size_t i = r.begin(); i != r.end(); i++) {
										if(!all_jobs_scheduled(new_states[i]) || (current_job_count!=jobs.size())) {
											explore(new_states[i]);
										}
									}
							});
						}
					});

#else
					for (const State& s : exploration_front) {
						if(!all_jobs_scheduled(s) || (current_job_count!=jobs.size()))
						{
							explore(s);
						}

						check_cpu_timeout();
						if (aborted){
							break;
						}
					}
#endif
					current_job_count++;

					// clean up the state cache if necessary
					if(is_aborted) {
						aborted = true;
						break;
					}

					if (!be_naive)
					{
						states_by_key.clear();
					}

#ifdef CONFIG_PARALLEL
					// propagate any updates to the response-time estimates
					for (auto& r : partial_rta)
						for (const auto& elem : r)
							update_finish_times(rta, elem.first, elem.second);
#endif
#ifndef CONFIG_COLLECT_SCHEDULE_GRAPH
					/* If we don't need to collect all states, we can remove
					 all those that we are done with, which saves a lot of
					 memory.*/
#ifdef CONFIG_PARALLEL
					parallel_for(states_storage.front().range(),
						[] (typename Split_states::range_type& r) {
							for (auto it = r.begin(); it != r.end(); it++) {
								it->clear();
							}
						});
#endif
					states_storage.pop_front();
#endif
				}


#ifndef CONFIG_COLLECT_SCHEDULE_GRAPH
				// clean out any remaining states
				while (!states_storage.empty()) {
#ifdef CONFIG_PARALLEL
					parallel_for(states_storage.front().range(),
						[] (typename Split_states::range_type& r) {
							for (auto it = r.begin(); it != r.end(); it++)
								it->clear();
						});
#endif
					states_storage.pop_front();
				}
#endif

#ifdef CONFIG_PARALLEL
				for (auto &c : edge_counter) {
					num_edges += c;
				}
#endif
			}

#ifdef CONFIG_COLLECT_SCHEDULE_GRAPH
			friend std::ostream& operator<< (std::ostream& out,
			                                 const State_space<Time>& space)
			{
					std::map<const Schedule_state<Time>*, unsigned int> state_id;
					unsigned int i = 0;
					out << "digraph {" << std::endl;
#ifdef CONFIG_PARALLEL
					for (const Split_states& states : space.get_states()) {
						for (const Schedule_state<Time>& s : tbb::flattened2d<Split_states>(states)) {
#else
					for (const auto& front : space.get_states()) {
						for (const Schedule_state<Time>& s : front) {
#endif
							state_id[&s] = i++;
							out << "\tS" << state_id[&s]
								<< "[label=\"S" << state_id[&s] << ": ";
							s.print_vertex_label(out, space.jobs);
							out << "\"];" << std::endl;
						}
					}
					if(!is_heterogeneous) {
						for (const auto& e : space.get_edges()) {
							out << "\tS" << state_id[e.source]
								<< " -> "
								<< "S" << state_id[e.target]
								<< "[label=\""
								<< "T" << e.scheduled->get_task_id()
								<< " J" << e.scheduled->get_job_id()
								<< "\\nDL=" << e.scheduled->get_deadline()
								<< "\\nES=" << e.earliest_start_time()
								<< "\\nLS=" << e.latest_start_time()
								<< "\\nEF=" << e.earliest_finish_time()
								<< "\\nLF=" << e.latest_finish_time()
								<< "\\nKey=" << e.StateKey
								<< "\"";
							if (e.deadline_miss_possible()) {
								out << ",color=Red,fontcolor=Red";
							}
							out << ",fontsize=8" << "]"
								<< ";"
								<< std::endl;
							if (e.deadline_miss_possible()) {
								out << "S" << state_id[e.target]
									<< "[color=Red];"
									<< std::endl;
							}
						}
					} else {
						// For non-batch jobs
						for (const auto& e : space.get_edges()) {
							out << "\tS" << state_id[e.source]
								<< " -> "
								<< "S" << state_id[e.target]
								<< "[label=\""
								<< "T" << e.scheduled->get_task_id()
								<< " J" << e.scheduled->get_job_id()
								<< "\nRSC=" << e.RSC
								<< "\\nDL=" << e.scheduled->get_deadline()
								<< "\\nES=" << e.earliest_start_time()
								<< "\\nLS=" << e.latest_start_time()
								<< "\\nEF=" << e.earliest_finish_time()
								<< "\\nLF=" << e.latest_finish_time()
								<< "\\nKey=" << e.StateKey
								<< "\"";
							if (e.deadline_miss_possible()) {
								out << ",color=Red,fontcolor=Red";
							}
							out << ",fontsize=8" << "]"
								<< ";"
								<< std::endl;
							if (e.deadline_miss_possible()) {
								out << "S" << state_id[e.target]
									<< "[color=Red];"
									<< std::endl;
							}
						}
					}
					out << "}" << std::endl;
				return out;
			}
#endif
		};
	}
}

namespace std
{
	template<class Time> struct hash<NP::Global::Schedule_state<Time>>
    {
		std::size_t operator()(NP::Global::Schedule_state<Time> const& s) const
        {
            return s.get_key();
        }
    };
}

#endif
