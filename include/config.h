#ifndef CONFIG_H
#define CONFIG_H

#define SELF_SUSPENSION_TYPE 	0
#define SUSPENSION_CORES 		0
#define STATE_CACHE_SIZE		50
#define MAX_JOBS_IN_HP 			65535
#define DUMMY_NUMBER			999999999
#define DUMMY_RESOURCE_TYPE 	DUMMY_NUMBER
#define DUMMY_TIME_VALUE		DUMMY_NUMBER
#define DUMMY_JOB_INDEX			DUMMY_NUMBER

// DM : debug message -- disable for now
//#define DM(x) std::cerr << x
#define DM(x)

#define ENABLE_STATE_CACHING
#define PESSIMISM_REDUCTION
//#define CONFIG_COLLECT_SCHEDULE_GRAPH

#ifndef CONFIG_COLLECT_SCHEDULE_GRAPH
#define CONFIG_PARALLEL
#endif

#ifndef NDEBUG
#define TBB_USE_DEBUG 1
#endif

#endif
